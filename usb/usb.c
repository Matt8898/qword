#include <usb/usb.h>
#include <usb/hcd/xhci.h>
#include <lib/dynarray.h>

dynarray_new(struct usb_dev_t, usb_devs);

void init_usb(void) {
	struct usb_hc_t *xhci_controller = usb_init_xhci();
}
