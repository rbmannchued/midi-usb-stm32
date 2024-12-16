#include <stdlib.h>
#include <stdint.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencmsis/core_cm3.h>
static usbd_device *usbd_dev;


void usb_isr(usbd_device *dev, uint8_t ep);


static const struct usb_device_descriptor dev_descr = {
        /* Type: uint8_t   Size: 1   
         * Description: Size of this descriptor in bytes
         */
        .bLength = USB_DT_DEVICE_SIZE,

        /* Type: uint8_t   Size: 1   
         * Descriptor: Device Descriptor Type = 1
         */
    	.bDescriptorType = USB_DT_DEVICE,  

        /* Type: uint16_t   Size: 2   
         * Description: This field identifies the release of the USB 
         * Specification with which the device and its descriptors are 
         * compliant. 
         */
        .bcdUSB = 0x0200,

        /* Type: uint8_t   Size: 1   
         * Description: Class code (assigned by the USB-IF)   0 = each 
         * interface within a configuration specifies its own class information 
         * and the various interfaces operate independently.
         */
    	.bDeviceClass = 0,

        /* Type: uint8_t   Size: 1   
         * Description: Subclass code (assigned by the USB-IF)   
         * if bDeviceClass = 0 then bDeviceSubClass = 0
         */
	    .bDeviceSubClass = 0,

        /* Type: uint8_t   Size: 1   
         * Description: Protocol code (assigned by the USB-IF)   
         * 0 = the device does not use class specific protocols on a device 
         * basis. However, it may use class specific protocols on an interface 
         * basis
         */
    	.bDeviceProtocol = 0,

        /* Type: uint8_t   Size: 1   
         * Description: Maximum packet size for Endpoint zero 
         * (only 8, 16, 32, or 64 are valid)
         */
	    .bMaxPacketSize0 = 64,

        /* Type: uint16_t   Size: 2   
         * Description: Vendor ID (assigned by the USB-IF)
         */
	    .idVendor = 0x0666,

        /* Type: uint16_t   Size: 2   
         * Description: Product ID (assigned by the manufacturer)
         */
	    .idProduct = 0x0815,

        /* Type: uint16_t   Size: 2   
         * Description: Device release number in binary-coded decimal
         */
	    .bcdDevice = 0x0101,

        /* Type: uint8_t   Size: 1   
         * Description: Index of string descriptor describing manufacturer
         */
	    .iManufacturer = 1,

        /* Type: uint8_t   Size: 1   
         * Description: Index of string descriptor describing product
         */
	    .iProduct = 2,

        /* Type: uint8_t   Size: 1   
         * Description: Index of string descriptor describing the device's 
         * serial number
         */
	    .iSerialNumber = 3,

        /* Type: uint8_t   Size: 1   
         * Description: Number of possible configurations
         */
	    .bNumConfigurations = 1,  
};
static void usb_setup(usbd_device *dev, uint16_t wValue)
{

	(void)wValue;

	/* Setup USB Receive interrupt. */
    usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();

}

//{ poll USB on interruptg */
void usb_lp_can_rx0_isr(void) {
    	usbd_poll(usbd_dev);
}


/* Midi specific endpoint descriptors. */
static const struct usb_midi_endpoint_descriptor midi_usb_endp[] = {
    {
                
	/* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
         */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
}, {
        /* Table B-14: MIDI Adapter Class-specific Bulk IN Endpoint
         * Descriptor 
             */
        .head = {
            .bLength = sizeof(struct usb_midi_endpoint_descriptor),
            .bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
            .bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
            .bNumEmbMIDIJack = 1,
        },
        .jack[0] = {
            .baAssocJackID = 0x03,
        },
    } 
};


/* Standard endpoint descriptors */
static const struct usb_endpoint_descriptor usb_endp[] = {
    {
            /* Look above */
        .bLength = USB_DT_ENDPOINT_SIZE,

            /* Look above */
        .bDescriptorType = USB_DT_ENDPOINT,

             /* Look above */
        .bEndpointAddress = 0x81,

            /* Look above */
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,

            /* Look above */
        .wMaxPacketSize = 0x40,

            /* Look above */
        .bInterval = 0x00, 			 

            /* Look above */
        .extra = &midi_usb_endp[1],

            /* Look above */
        .extralen = sizeof(midi_usb_endp[1]), 
    } 
};


/* Table B-4: MIDI Adapter Class-specific AC Interface Descriptor */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) +
		           1 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength =
			   sizeof(struct usb_audio_header_descriptor_head) +
			   1 * sizeof(struct usb_audio_header_descriptor_body),
		.binCollection = 1,
	},
	.header_body = {
		.baInterfaceNr = 0x01,
	},
};


/* MIDI Adapter Standard AC Interface Descriptor */
static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &audio_control_functional_descriptors,
	.extralen = sizeof(audio_control_functional_descriptors)
} };


/* Class-specific MIDI streaming interface descriptor */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_in_jack_descriptor in_embedded;
//	struct usb_midi_in_jack_descriptor in_external;
//	struct usb_midi_out_jack_descriptor out_embedded;
//	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
	/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
	.header = {
		.bLength = sizeof(struct usb_midi_header_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
		.bcdMSC = 0x0100,
		.wTotalLength = sizeof(midi_streaming_functional_descriptors),
	},
	/* Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) */
	.in_embedded = {
		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
		.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
		.bJackID = 0x01,
		.iJack = 0x00,
	},
	/* Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) */
//	.in_external = {
//		.bLength = sizeof(struct usb_midi_in_jack_descriptor),
//		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
//		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_IN_JACK,
//		.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
//		.bJackID = 0x02,
//		.iJack = 0x00,
//	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
//	.out_embedded = {
//		.head = {
//			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
//			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
//			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
//			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
//			.bJackID = 0x03,
//			.bNrInputPins = 1,
//		},
//		.source[0] = {
//			.baSourceID = 0x02,
//			.baSourcePin = 0x01,
//		},
//		.tail = {
//			.iJack = 0x00,
//		}
//	},
//	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
//	.out_external = {
//		.head = {
//			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
//			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
//			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
//			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
//			.bJackID = 0x04,
//			.bNrInputPins = 1,
//		},
//		.source[0] = {
//			.baSourceID = 0x01,
//			.baSourcePin = 0x01,
//		},
//		.tail = {
//			.iJack = 0x00,
//		},
//	},
};


/* Table B-5: MIDI Adapter Standard MS Interface Descriptor */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = usb_endp,

	.extra = &midi_streaming_functional_descriptors,
	.extralen = sizeof(midi_streaming_functional_descriptors)
} };

/* Struct with the Interface Descriptors */
static const struct usb_interface ifaces[] = {
        {
                /* Audio Control Interface Descriptor*/
                .num_altsetting = 1,
                .altsetting = audio_control_iface,
        },
        {
                /* MIDI Streaming Interface Descriptor*/
                .num_altsetting = 1,
                .altsetting = midi_streaming_iface,
        }
};


/* Configuration Descriptor */
static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,

        /* can be anything, it is updated automatically
         * when the usb code prepares the descriptor 
         */
	.wTotalLength = 0,

        /* control and data */
	.bNumInterfaces = 2, 
	.bConfigurationValue = 1,
	.iConfiguration = 0,

        /* bus powered (power settings)*/
	.bmAttributes = 0x80, 
	.bMaxPower = 0x64,

	.interface = ifaces,
};


/* USB Strings (Look at USB-Device-Descriptor) */
static const char * usb_strings[] = {

        /* Manufacturer */
	"Rafael Bormann Chuede",

        /* Product */
	"MIDI STM32",

        /* SerialNumber */
	"AHSM00003\0"			
};


/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];


/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {

        /* USB Framing (3 byte SysEx) */
	0x04,

        /* SysEx start */
	0xf0,

        /* non-realtime */
	0x7e,

        /* Channel 0 */
	0x00,

        /* USB Framing (3 byte SysEx) */
	0x04,

        /* Educational/prototype manufacturer ID */
	0x7d,

        /* Family code (byte 1) */
	0x66,

        /* Family code (byte 2) */
	0x66,

        /* USB Framing (3 byte SysEx) */
	0x04,

        /* Model number (byte 1) */
	0x51,

        /* Model number (byte 2) */
	0x19,

        /* Version number (byte 1) */
	0x00,

        /* USB Framing (3 byte SysEx) */
	0x04,

        /* Version number (byte 2) */
	0x00,

        /* Version number (byte 3) */
	0x01,

        /* Version number (byte 4) */
	0x00,

        /* USB Framing (1 byte SysEx) */
	0x05,

        /* SysEx end */
	0xf7,

        /* Padding */
	0x00,

        /* Padding */
	0x00,	
};
//typedef struct FIFO{
//        /* Read pointer */
//	uint8_t *read;
//
//        /* Write pointer */
//	uint8_t *write;
//
//        /* Size of the FIFO */
//	size_t size;
//
//        /* Start adress pointer */
//	uint8_t *start;
//
//        /* End adress pointer */
//	uint8_t *end;
//
//        /* current data (read return) */
//	uint8_t data;
//
//        /* is the FIFO empty -> 1=yes 0=no */
//	uint8_t empty;
//
//        /* number of midi commands (1 command = 3 8bit) */
//	uint8_t midi_commands;
//}FIFO;

/* USB FIFO */
//static FIFO usb_FIFO;


/* FIFO Setup */
//static FIFO FIFO_setup(FIFO fifo, size_t size){
//    fifo.size = size;
//    fifo.start = malloc(fifo.size * sizeof(uint8_t));    
//    fifo.end = fifo.start + size;
//    fifo.write = fifo.start;
//    fifo.read = fifo.start;
//    return fifo;
//}

/* FIFO Write */
//static FIFO FIFO_write(FIFO fifo, uint8_t data){
//    if(fifo.write == fifo.end){
//        if(fifo.read != fifo.start){
//            fifo.write = fifo.start;
//        } else {
//            /* FIFO full */
//            return fifo;
//        }
//    } else {
//        if((fifo.write + 1) != fifo.read){ 
//            fifo.write = fifo.write + 1;
//        } else {
//            /* FIFO full */
//            return fifo;
//        }
//    }
//    *fifo.write = data;
//    return fifo;
//}

/* FIFO Read */
//static FIFO FIFO_read(FIFO fifo){
//    if(fifo.read == fifo.end){
//        if(fifo.write != fifo.end){
//            fifo.read = fifo.start;
//        } else {
//            /* FIFO empty */
//            fifo.empty = 1;
//            return fifo;
//        }
//    } else {
//        if(fifo.read != fifo.write){
//            fifo.read = fifo.read + 1;
//        } else {
//            /* FIFO empty */
//            fifo.empty = 1;
//            return fifo;
//        }
//    }
//    fifo.data = *fifo.read;
//    fifo.empty = 0;		 
//    return fifo;
//}
//    void usb_isr(usbd_device *dev, uint8_t ep){
//        (void)ep;	
//        char buf[64];
//            
//            usbd_ep_read_packet(dev, 0x01, buf, 64);
//            
//            usb_FIFO = FIFO_write(usb_FIFO, buf[1]); /* MIDI command */
//            usb_FIFO = FIFO_write(usb_FIFO, buf[2]); /* MIDI note */
//            usb_FIFO = FIFO_write(usb_FIFO, buf[3]); /* MIDI velocity */
//            usb_FIFO.midi_commands++;
//    }
static void usb_send(usbd_device *dev){
    /* Prepare MIDI packet for Note On message */
    char buf[4] = {
        0x08,                   /* Command (Note On) */
        0x90,                   /* MIDI channel (Note On with channel 0) */
        0x3C,                   /* Note number (middle C) */
        0x7F                    /* Velocity (max) */
    };

    while (usbd_ep_write_packet(dev, 0x81, buf, sizeof(buf)) == 0);
}

static void loop(void){
    while(1){
        usb_send(usbd_dev);
        for (int i = 0; i < 0x800000; i++) { __asm__("nop"); } // Small delay
    }
}

int main(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_GPIOB);

//    static FIFO usb_FIFO;
    /* FIFO Setup */
    //usb_FIFO = FIFO_setup(usb_FIFO, 64);

    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr,
                         &config, usb_strings, 3, usbd_control_buffer,
                         sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, usb_setup);

    /* Wait for USB to register on the PC */
    for (int i = 0; i < 0x800000; i++) { __asm__("nop"); }

    /* Wait for USB Vbus. */
    while (gpio_get(GPIOA, GPIO8) == 0) { __asm__("nop"); }

    loop();
}

