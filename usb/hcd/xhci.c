#include <usb/hcd/xhci.h>
#include <sys/idt.h>
#include <lib/klib.h>
#include <lib/alloc.h>
#include <lib/dynarray.h>
#include <lib/cmem.h>
#include <sys/pci.h>
#include <sys/panic.h>
#include <lib/lock.h>
#include <lib/cio.h>
#include <mm/mm.h>
#include <sys/apic.h>

static int xhci_setup_seg(struct xhci_seg *seg, size_t size, uint32_t type) {
    seg->trbs = kalloc(size);
    if(!seg->trbs) {
        return 0;
    }
    seg->trbs_dma = (size_t)seg->trbs - MEM_PHYS_OFFSET;

    seg->size = size / 16;
    seg->next = NULL;
    seg->type = type;
    seg->enq = (size_t)seg->trbs;
    seg->deq = (size_t)seg->trbs;
    memset((void *)seg->trbs, 0, size);
    //The link trb points to the address of the next trb ring
    //in our case we are jut going to make it loop.
    struct xhci_link_trb *link;

    if(type != TYPE_EVENT) {
        link =(struct xhci_link_trb *) (seg->trbs + seg->size - 1);
        link->addr = (size_t)seg->trbs - MEM_PHYS_OFFSET;
        link->interrupter_target = 0;
        link->flags = (0x1 | TRB_CMD_TYPE(TRB_LINK));
    }

    return 1;
}

void xhci_irq_handler(struct xhci_hcd *controller) {
    for (;;) {
        event_await(&int_event[controller->irq_line]);

        kprint(KPRN_INFO, "GOT IRQ");
        #define TRB_TYPE(x)          (((x) & (0x3F << 10)) >> 10)

        //handle events
        volatile struct xhci_event_trb *event = (struct xhci_event_trb*)controller->ering.deq;
        kprint(KPRN_INFO, "usb/xhci: event deq: %X", event);
        size_t deq = 0;
        while((event->flags & 0x1) != controller->ering.cycle_state) {
            kprint(KPRN_INFO, "usb/xhci: TRB type: %X", TRB_TYPE(event->flags));

            switch(TRB_TYPE(event->flags)) {
                case 33: {
                    kprint(KPRN_INFO, "usb/xhci: Got command completion");
                    int index = (event->addr - ((size_t)controller->crseg.trbs - MEM_PHYS_OFFSET)) / 0x10;
                    kprint(KPRN_INFO, "usb/xhci: dequeueing event at index %X", index);
                    controller->crseg.seg_events[index]->trb = *event;
                    event_trigger(&controller->crseg.seg_events[index]->event);
                    break;
                }
            }

            controller->ering.deq = (uint64_t) (event + 1);
            int index = controller->ering.deq - (uint64_t)controller->ering.trbs;
            uint64_t val = (size_t)controller->ering.trbs - MEM_PHYS_OFFSET;
            val += (index % 4096);
            if(!(index % 4096)) {
                controller->ering.deq = (uint64_t)controller->ering.trbs;
                controller->ering.cycle_state = !controller->ering.cycle_state;
            }

            event = (struct xhci_event_trb*)controller->ering.deq;
            deq = (size_t)controller->ering.deq - MEM_PHYS_OFFSET;
        }

        controller->run_regs->irs[0].iman = controller->run_regs->irs[0].iman | 1;
        controller->run_regs->irs[0].erdp = deq | (1 << 3);
    }
}

/*
 * poll for events and simulate an irq, this is a workaround.
 */
void xhci_wait_event(struct xhci_hcd *controller) {
    //event_await(&int_event[controller->irq_line]);

    #define TRB_TYPE(x)          (((x) & (0x3F << 10)) >> 10)

    //handle events
    volatile struct xhci_event_trb *event = (struct xhci_event_trb*)controller->ering.deq;
    kprint(KPRN_INFO, "usb/xhci: waiting for event at: %X", event);

    while((event->flags & 0x1) == controller->ering.cycle_state) {
    }

    kprint(KPRN_INFO, "usb/xhci: event deq: %X", event);
    size_t deq = 0;
    while((event->flags & 0x1) != controller->ering.cycle_state) {
        kprint(KPRN_INFO, "usb/xhci: TRB type: %X", TRB_TYPE(event->flags));

        switch(TRB_TYPE(event->flags)) {
            case 33: {
                kprint(KPRN_INFO, "usb/xhci: Got command completion");
                int index = (event->addr - ((size_t)controller->crseg.trbs - MEM_PHYS_OFFSET)) / 0x10;
                kprint(KPRN_INFO, "usb/xhci: dequeueing event at index %X", index);
                controller->crseg.seg_events[index]->trb = *event;
                event_trigger(&controller->crseg.seg_events[index]->event);
                break;
            }
        }

        //Advance the event pointer.
        controller->ering.deq = (uint64_t) (event + 1);
        int index = controller->ering.deq - (uint64_t)controller->ering.trbs;
        uint64_t val = (size_t)controller->ering.trbs - MEM_PHYS_OFFSET;
        val += (index % 4096);
        if(!(index % 4096)) {
            controller->ering.deq = (uint64_t)controller->ering.trbs;
            controller->ering.cycle_state = !controller->ering.cycle_state;
        }

        event = (struct xhci_event_trb*)controller->ering.deq;
        deq = (size_t)controller->ering.deq - MEM_PHYS_OFFSET;
    }
}

static int xhci_send_command(struct xhci_hcd *controller, uint32_t field1, uint32_t field2, uint32_t field3, uint32_t field4, struct xhci_event *ev) {
    volatile struct xhci_db_regs *dbr;
    volatile struct xhci_command_trb *cmd;
    uint32_t val, cycle_state;

    dbr = controller->db_regs;
    cmd = (struct xhci_command_trb *)controller->crseg.enq;

    cmd->field[0] = field1;
    cmd->field[1] = field2;
    cmd->field[2] = field3;

    val = cmd->field[3];
    cycle_state = (val & 0x1) ? 0 : 1;
    val = field4 | cycle_state;
    cmd->field[3] = val;

    /* Ring the doorbell */
    int index = (controller->crseg.enq - (uint64_t)controller->crseg.trbs) / 0x10;
    kprint(KPRN_INFO, "usb/xhci: enqueuing comannd at %X", index);
    //Set the event pointer
    controller->crseg.seg_events[index] = ev;
    dbr->db[0] = 0;

    cmd++;
    controller->crseg.enq = (uint64_t)cmd;
    //return ret;
    return 0;
}

void xhci_enable_slot(struct xhci_hcd *controller) {
    ksleep(10);
    uint32_t field1 = 0, field2 = 0, field3 = 0, field4 = TRB_CMD_TYPE(TRB_ENABLE_SLOT);
    struct xhci_event ev = {0};
    xhci_send_command(controller, field1, field2, field3, field4, &ev);
 //   xhci_wait_event(controller);
    event_await(&ev.event);
    if(TRB_STATUS(ev.trb.status)) {
        int slot = TRB_SLOT_ID(ev.trb.flags);
        kprint(KPRN_INFO, "usb/xhci: slot %x has been enabled", slot);
        controller->slot_id = slot;
    } else {
        kprint(KPRN_INFO, "usb/xhci: failed to enable slot");
        controller->slot_id = 0;
    }
}

static void xhci_send_addr_device(struct xhci_hcd *xhcd, uint32_t slot_id, uint64_t dma_in_ctx, int bsr) {
    uint32_t field1, field2, field3, field4;

    field1 = TRB_ADDR_LOW(dma_in_ctx) & ~0xF;
    field2 = TRB_ADDR_HIGH(dma_in_ctx);
    field3 = 0;
    field4 = TRB_CMD_TYPE(TRB_ADDRESS_DEV) | TRB_CMD_SLOT_ID(slot_id) | (bsr << 9);
    struct xhci_event ev = {0};
    xhci_send_command(xhcd, field1, field2, field3, field4, &ev);
    event_await(&ev.event);
    if((ev.trb.status >> 24) != 1) {
        kprint(KPRN_INFO, "usb/xhci: Error while attempting to address device");
    } else {
        kprint(KPRN_INFO, "usb/xhci: Device addressed");
    }
}

static void xhci_configure_ep(struct xhci_hcd *xhcd, uint32_t slot_id,
                              uint64_t dma_in_ctx) {
    uint32_t field1, field2, field3, field4;

    field1 = TRB_ADDR_LOW(dma_in_ctx) & ~0xF;
    field2 = TRB_ADDR_HIGH(dma_in_ctx);
    field3 = 0;
    field4 = TRB_CMD_TYPE(TRB_CONFIG_EP) | TRB_CMD_SLOT_ID(slot_id);
    struct xhci_event ev = {0};
    xhci_send_command(xhcd, field1, field2, field3, field4, &ev);
    event_await(&ev.event);
    if((ev.trb.status >> 24) != 1) {
        kprint(KPRN_INFO, "usb/xhci: Error while attempting to configure endpoint");
    } else {
        kprint(KPRN_INFO, "usb/xhci: Endpoint configured");
    }
}

/*
 * Context handling
 */

static struct xhci_ep_ctx *xhci_get_ep0_ctx(struct xhci_ctx *ctx, uint32_t ctx_size) {
    uint32_t offset = ctx_size;
    int mul = 1;

    offset = ctx_size;
    if (ctx->type == XHCI_CTX_TYPE_INPUT) {
        mul++;
    }
    return (struct xhci_ep_ctx *)(ctx->addr + offset * mul);
}

static struct xhci_ep_ctx *xhci_get_ep_ctx(volatile struct xhci_ctx *ctx, uint32_t ctx_size,
                                           uint32_t epno) {
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        epno++;
    return (struct xhci_ep_ctx *)(ctx->addr + epno * ctx_size);
}

static struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_ctx *ctx, uint32_t ctx_size) {
    uint32_t offset = 0;
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        offset += ctx_size;
    return (struct xhci_slot_ctx *)((size_t)ctx->addr + offset);
}

static struct xhci_control_ctx *xhci_get_control_ctx(struct xhci_ctx *ctx) {
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        return (struct xhci_control_ctx *) ctx->addr;
    return NULL;
}

void xhci_setup_context(struct xhci_ctx *ctx, uint32_t size, uint32_t type) {
    void* addr = kalloc(size);
    ctx->addr = (uint8_t*)addr;
    ctx->dma_addr = (size_t)ctx->addr - MEM_PHYS_OFFSET;
    ctx->type = type;
    ctx->size = size;
}

void xhci_init_device(struct xhci_hcd *controller, uint32_t port, uint32_t usb_ver) {
    kprint(KPRN_INFO, "usb/xhci: initializing device on port %X", port);
    xhci_enable_slot(controller);
    if(!controller->slot_id) {
        //TODO better error handling here
        return;
    }

    uint32_t slot_id = controller->slot_id;

    volatile struct usb_dev_t device;
    volatile struct xhci_dev*  xhci_dev = kalloc(sizeof(struct xhci_dev));
    volatile struct xhci_slot_ctx *slot;
    volatile struct xhci_control_ctx *ctrl;
    volatile struct xhci_ep_ctx *ep0;
    uint32_t ctx_size, val;
    uint16_t max_packet;
    uint32_t newport, rootport;

    /*
     * Set up contexts
     */
    xhci_setup_context(&xhci_dev->in_ctx, 4096, XHCI_CTX_TYPE_INPUT);
    xhci_setup_context(&xhci_dev->out_ctx, 4096, XHCI_CTX_TYPE_DEVICE);

    //the control context determines which endpoint contextes should be evaluated
    ctrl = xhci_get_control_ctx(&xhci_dev->in_ctx);
    //Add flags
    ctrl->a_flags = 1;
    //Drop flags
    ctrl->d_flags = 0;

    //The slot context determines parameters relating to the device, the slot or all endpoints in general
    slot = xhci_get_slot_ctx(&xhci_dev->in_ctx, controller->context_size);

    //TODO support non-root hub ports
    #define ROOT_HUB_PORT(x)        ((x & 0xff) << 16)
    newport = rootport = port + 1;
    slot->field1 = (1 << 27) |  ((0x4 << 10) << 10);
    slot->field2 = ROOT_HUB_PORT(port + 1);//PORT SUPPORT HUBS at some point

    //Device control endpoint
    xhci_setup_seg(&xhci_dev->control, 4096, TYPE_CTRL);
    ep0 = xhci_get_ep0_ctx(&xhci_dev->in_ctx, controller->context_size);
    //TODO handle non-usb3 devices, for usb 2 and usb 1 low speed devices this can be assumed
    //full speed ones need some parsing to be done.
    if(usb_ver == 3) {
        max_packet = 512;
    }

    val = EP_TYPE(EP_CTRL) | MAX_BURST(0) | ERROR_COUNT(3) | MAX_PACKET_SIZE(max_packet);
    ep0->field2 = (val);
    ep0->deq_addr = (xhci_dev->control.trbs_dma | xhci_dev->control.cycle_state);

    controller->dcbaap[slot_id] = xhci_dev->out_ctx.dma_addr;
    slot = xhci_get_slot_ctx(&xhci_dev->out_ctx, controller->context_size);
    ep0 = xhci_get_ep0_ctx(&xhci_dev->in_ctx, controller->context_size);
    xhci_send_addr_device(controller, slot_id, xhci_dev->in_ctx.dma_addr, 0);
    xhci_configure_ep(controller, slot_id, xhci_dev->in_ctx.dma_addr);
    while (1){}
}

static void xhci_scan_ports(struct xhci_hcd *controller) {
    uint32_t hccparams = controller->cap_regs->hccparams1;
    //uint32_t maxp_ports = hccparams >> 24;
    //TODO this only works for usb 3 devices for now.
    //also actually check the amount of ports.
    //usb 2.0 devices need to be reset.
    for(int i = 0; i < 32; i++) {
        volatile struct xhci_port_regs *port_regs1 = &controller->op_regs->prs[i];
        if(port_regs1->portsc & 0x1) {
            while(1) {
                if((((port_regs1->portsc >> 5) & 0xf) == 0)) {
                    //break;
                    //The port can be initialized.
                    xhci_init_device(controller, i, 3);
                }
            }
        }
    }
}

void xhci_controller_init(struct pci_device_t* pci_dev) {
    struct pci_bar_t bar = {0};

    panic_if(pci_read_bar(pci_dev, 0, &bar));
    panic_unless(bar.is_mmio);

    panic_unless((pci_read_device_dword(pci_dev, 0x10) & 0b111) == 0b100);

    struct xhci_hcd *controller = kalloc(sizeof(struct xhci_hcd));
    size_t base = bar.base + MEM_PHYS_OFFSET;
    controller->cap_regs = (struct xhci_cap_regs *)(base);
    controller->op_regs = (struct xhci_op_regs *)(base + (controller->cap_regs->caplength));
    controller->run_regs = (struct xhci_run_regs *)(base + (controller->cap_regs->rtsoff));
    controller->db_regs = (struct xhci_db_regs *)(base + (controller->cap_regs->dboff));

    int32_t cmd = controller->op_regs->usbcmd;
    cmd &= ~0x1;
    controller->op_regs->usbcmd = cmd | 1 << 1;
    //Wait for controller not ready
    while(!(controller->op_regs->usbsts & 0x00000001UL)){};
    kprint(KPRN_INFO, "usb/xhci: controller halted");

    controller->op_regs->config = XHCI_CONFIG_MAX_SLOT;
    uint32_t hccparams = controller->cap_regs->hccparams1;
    if(hccparams & 0b10) {
        controller->context_size = 64;
    } else {
        controller->context_size = 32;
    }

    controller->dcbaap = kalloc(2048);
    controller->dcbaap_dma = (size_t)controller->dcbaap - MEM_PHYS_OFFSET;
    controller->op_regs->dcbaap = controller->dcbaap_dma;

    //Set up scratchpad_buffers
    uint32_t hcs2 = controller->cap_regs->hcsparams2;
    int spb = ((((hcs2) >> 16) & 0x3e0) | (((hcs2) >> 27) & 0x1f));
    if(spb) {
        controller->scratchpad_buffer_array = kalloc(sizeof(uint64_t) * spb);
        kprint(KPRN_INFO, "usb/xhci: allocating %x scratchpad_buffers", spb);
        for(int i = 0; i < spb; i++) {
            size_t scratchpad_buffer = (size_t)kalloc(PAGE_SIZE);
            controller->scratchpad_buffer_array[i] = scratchpad_buffer - MEM_PHYS_OFFSET;
        }
    }

    controller->dcbaap[0] = (size_t)controller->scratchpad_buffer_array - MEM_PHYS_OFFSET;
    xhci_setup_seg(&controller->crseg, 4096, TYPE_COMMAND);
    controller->op_regs->crcr = controller->crseg.trbs_dma | 1;
    kprint(KPRN_INFO, "usb/xhci: Initializing event ring");
    xhci_setup_seg(&controller->ering, 4096, TYPE_EVENT);
    //Set up event ring segment table.
    controller->erst.entries = kalloc(4096);
    controller->erst.dma = (size_t)controller->erst.entries - MEM_PHYS_OFFSET;
    controller->erst.num_segs = 1;
    controller->erst.entries->addr = controller->ering.trbs_dma;
    controller->erst.entries->size = controller->ering.size;
    controller->erst.entries->reserved = 0;

    kprint(KPRN_INFO, "usb/xhci: erdp %X", controller->ering.trbs_dma);
    //Set up erdp
    controller->run_regs->irs[0].erdp = controller->ering.trbs_dma;
    controller->run_regs->irs[0].iman = 1 << 1;

    kprint(KPRN_INFO, "usb/xhci: erstsz");
    //Set up erstsz
    controller->run_regs->irs[0].erstsz = controller->erst.num_segs;

    kprint(KPRN_INFO, "usb/xhci: erstba %X", controller->erst.dma);
    //Set erstba
    controller->run_regs->irs[0].erstba = controller->erst.dma;
    controller->run_regs->irs[0].iman |= 0x10;

    controller->irq_line = get_empty_int_vector();
    if(!pci_register_msi(pci_dev, controller->irq_line)) {
        io_apic_connect_gsi_to_vec(0, controller->irq_line, pci_dev->gsi, pci_dev->gsi_flags, 1);
    }

    task_tcreate(0, tcreate_fn_call, tcreate_fn_call_data(0, xhci_irq_handler, controller));

    cmd = controller->op_regs->usbcmd;
    cmd |= 0x1 | 1 << 2;
    controller->op_regs->usbcmd = cmd;

    //Wait for controller not ready
    while((controller->op_regs->usbsts & 0x1)){};
    kprint(KPRN_INFO, "usb/xhci: conroller restarted");
    xhci_scan_ports(controller);
}

struct usb_hc_t *usb_init_xhci(void) {
    struct usb_hc_t* xhci_hcd;
    struct pci_device_t* pci_dev = pci_get_device(XHCI_CLASS, XHCI_SUBCLASS, PROG_IF, 0);
    if(pci_dev) {
        pci_enable_busmastering(pci_dev);
        xhci_controller_init(pci_dev);
    }
}
