/*===========================================================================
FILE:
   GobiSerial.c

DESCRIPTION:
   Linux Qualcomm Serial USB driver Implementation

PUBLIC DRIVER FUNCTIONS:
   GobiProbe
   GobiOpen
   GobiClose
   GobiReadBulkCallback (if kernel is less than 2.6.25)
   GobiSuspend
   GobiResume (if kernel is less than 2.6.24)

Copyright (c) 2011, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
==========================================================================*/
//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------

#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/version.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,2,0 ))
#include <linux/module.h>
#endif

//---------------------------------------------------------------------------
// Global veriable and defination
//---------------------------------------------------------------------------

// Version Information
#define DRIVER_VERSION "2011-07-29-1026"
#define DRIVER_AUTHOR "Qualcomm Innovation Center"
#define DRIVER_DESC "GobiSerial"

#define NUM_BULK_EPS         1
#define MAX_BULK_EPS         6

// Debug flag
static int debug;

// Global pointer to usb_serial_generic_close function
// This function is not exported, which is why we have to use a pointer
// instead of just calling it.
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   void (* gpClose)(
      struct usb_serial_port *,
      struct file * );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   void (* gpClose)(
      struct tty_struct *,
      struct usb_serial_port *,
      struct file * );
#else // > 2.6.30
   void (* gpClose)( struct usb_serial_port * );
#endif

// DBG macro
#define DBG( format, arg... ) \
   if (debug == 1)\
   { \
      printk( KERN_INFO "GobiSerial::%s " format, __FUNCTION__, ## arg ); \
   } \

/*=========================================================================*/
// Function Prototypes
/*=========================================================================*/

// Attach to correct interfaces
static int GobiProbe(
   struct usb_serial * pSerial,
   const struct usb_device_id * pID );

// Start GPS if GPS port, run usb_serial_generic_open
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   int GobiOpen(
      struct usb_serial_port *   pPort,
      struct file *              pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
   int GobiOpen(
      struct tty_struct *        pTTY,
      struct usb_serial_port *   pPort,
      struct file *              pFilp );
#else // > 2.6.31
   int GobiOpen(
      struct tty_struct *        pTTY,
      struct usb_serial_port *   pPort );
#endif

// Stop GPS if GPS port, run usb_serial_generic_close
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   void GobiClose(
      struct usb_serial_port *,
      struct file * );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   void GobiClose(
      struct tty_struct *,
      struct usb_serial_port *,
      struct file * );
#else // > 2.6.30
   void GobiClose( struct usb_serial_port * );
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))

// Read data from USB, push to TTY and user space
static void GobiReadBulkCallback( struct urb * pURB );

#endif

// Set reset_resume flag
int GobiSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent );

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))

// Restart URBs killed during usb_serial_suspend
int GobiResume( struct usb_interface * pIntf );

#endif

/*=========================================================================*/
// Qualcomm Gobi 3000 VID/PIDs
/*=========================================================================*/
static struct usb_device_id GobiVIDPIDTable[] =
{
   { USB_DEVICE( 0x05c6, 0x920c ) },   // Gobi 3000 QDL device
   { USB_DEVICE( 0x05c6, 0x920d ) },   // Gobi 3000 Composite Device
   { USB_DEVICE( 0x03f0, 0x371d ) },   // Gobi 3000 HP un2430 Device
   { USB_DEVICE( 0x1410, 0xa021 ) },   // Novatel Wireless E396 - Gobi VID/PID
   { USB_DEVICE( 0x1410, 0xa023 ) },   // Novatel Wireless E346 - Gobi VID/PID
   { }                               // Terminating entry
};
MODULE_DEVICE_TABLE( usb, GobiVIDPIDTable );

/*=========================================================================*/
// Struct usb_serial_driver
// Driver structure we register with the USB core
/*=========================================================================*/
static struct usb_driver GobiDriver =
{
   .name       = "GobiSerial",
#if ((LINUX_VERSION_CODE < KERNEL_VERSION( 3,5,0 )))
   .probe      = usb_serial_probe,
   .disconnect = usb_serial_disconnect,
#endif
   .id_table   = GobiVIDPIDTable,
   .suspend    = GobiSuspend,
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))
   .resume     = GobiResume,
#else
   .resume     = usb_serial_resume,
#endif
   .supports_autosuspend = true,
};

/*=========================================================================*/
// Struct usb_serial_driver
/*=========================================================================*/
static struct usb_serial_driver gGobiDevice =
{
   .driver =
   {
      .owner     = THIS_MODULE,
      .name      = "GobiSerial driver",
   },
   .description         = "GobiSerial",
   .id_table            = GobiVIDPIDTable,
   .usb_driver          = &GobiDriver,
   .num_ports           = 1,
   .probe               = GobiProbe,
   .open                = GobiOpen,
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))
   .num_interrupt_in    = NUM_DONT_CARE,
   .num_bulk_in         = 1,
   .num_bulk_out        = 1,
   .read_bulk_callback  = GobiReadBulkCallback,
#endif
};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,4,0 ))
   static struct usb_serial_driver * const gGobiDevices[] = {
      &gGobiDevice, NULL
   };
#endif

//---------------------------------------------------------------------------
// USB serial core overridding Methods
//---------------------------------------------------------------------------

/*===========================================================================
METHOD:
   GobiProbe (Free Method)

DESCRIPTION:
   Attach to correct interfaces

PARAMETERS:
   pSerial    [ I ] - Serial structure
   pID        [ I ] - VID PID table

RETURN VALUE:
   int - negative error code on failure
         zero on success
===========================================================================*/
static int GobiProbe(
   struct usb_serial *             pSerial,
   const struct usb_device_id *    pID )
{
   // Assume failure
   int nRetval = -ENODEV;

   int nNumInterfaces;
   int nInterfaceNum;
   DBG( "\n" );

   // Test parameters
   if ( (pSerial == NULL)
   ||   (pSerial->dev == NULL)
   ||   (pSerial->dev->actconfig == NULL)
   ||   (pSerial->interface == NULL)
   ||   (pSerial->interface->cur_altsetting == NULL)
   ||   (pSerial->type == NULL) )
   {
      DBG( "invalid parameter\n" );
      return -EINVAL;
   }

   nNumInterfaces = pSerial->dev->actconfig->desc.bNumInterfaces;
   DBG( "Num Interfaces = %d\n", nNumInterfaces );
   nInterfaceNum = pSerial->interface->cur_altsetting->desc.bInterfaceNumber;
   DBG( "This Interface = %d\n", nInterfaceNum );

   if (nNumInterfaces == 1)
   {
      // QDL mode?
      if (nInterfaceNum == 1 || nInterfaceNum == 0)
      {
         DBG( "QDL port found\n" );
         nRetval = usb_set_interface( pSerial->dev,
                                      nInterfaceNum,
                                      0 );
         if (nRetval < 0)
         {
            DBG( "Could not set interface, error %d\n", nRetval );
         }
      }
      else
      {
         DBG( "Incorrect QDL interface number\n" );
      }
   }
   else
   {
      // Composite mode
      if (nInterfaceNum == 2)
      {
         DBG( "Modem port found\n" );
         nRetval = usb_set_interface( pSerial->dev,
                                      nInterfaceNum,
                                      0 );
         if (nRetval < 0)
         {
            DBG( "Could not set interface, error %d\n", nRetval );
         }
      }
      else if (nInterfaceNum == 3)
      {
         DBG( "GPS port found\n" );
         nRetval = usb_set_interface( pSerial->dev,
                                      nInterfaceNum,
                                      0 );
         if (nRetval < 0)
         {
            DBG( "Could not set interface, error %d\n", nRetval );
         }

         // Check for recursion
         if (pSerial->type->close != GobiClose)
         {
            // Store usb_serial_generic_close in gpClose
            gpClose = pSerial->type->close;
            pSerial->type->close = GobiClose;
         }
      }
      else
      {
         // Not a port we want to support at this time
         DBG( "Unsupported interface number\n" );
      }
   }

   if (nRetval == 0)
   {
      // Clearing endpoint halt is a magic handshake that brings
      // the device out of low power (airplane) mode
      // NOTE: FCC verification should be done before this, if required
      struct usb_host_endpoint * pEndpoint;
      int endpointIndex;
      int numEndpoints = pSerial->interface->cur_altsetting
                         ->desc.bInterfaceNumber;

      for (endpointIndex = 0; endpointIndex < numEndpoints; endpointIndex++)
      {
         pEndpoint = pSerial->interface->cur_altsetting->endpoint
                   + endpointIndex;

         if (pEndpoint != NULL
         &&  usb_endpoint_dir_out( &pEndpoint->desc ) == true)
         {
            int pipe = usb_sndbulkpipe( pSerial->dev,
                                        pEndpoint->desc.bEndpointAddress );
            nRetval = usb_clear_halt( pSerial->dev, pipe );

            // Should only be one
            break;
         }
      }
   }

   return nRetval;
}

/*===========================================================================
METHOD:
   GobiOpen (Free Method)

DESCRIPTION:
   Start GPS if GPS port, run usb_serial_generic_open

PARAMETERS:
   pTTY    [ I ] - TTY structure (only on kernels <= 2.6.26)
   pPort   [ I ] - USB serial port structure
   pFilp   [ I ] - File structure (only on kernels <= 2.6.31)

RETURN VALUE:
   int - zero for success
       - negative errno on error
===========================================================================*/
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
int GobiOpen(
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
int GobiOpen(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#else // > 2.6.31
int GobiOpen(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort )
#endif
{
   const char startMessage[] = "$GPS_START";
   int nResult;
   int bytesWrote;

   DBG( "\n" );

   // Test parameters
   if ( (pPort == NULL)
   ||   (pPort->serial == NULL)
   ||   (pPort->serial->dev == NULL)
   ||   (pPort->serial->interface == NULL)
   ||   (pPort->serial->interface->cur_altsetting == NULL) )
   {
      DBG( "invalid parameter\n" );
      return -EINVAL;
   }

   // Is this the GPS port?
   if (pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber == 3)
   {
      // Send startMessage, 1s timeout
      nResult = usb_bulk_msg( pPort->serial->dev,
                              usb_sndbulkpipe( pPort->serial->dev,
                                               pPort->bulk_out_endpointAddress ),
                              (void *)&startMessage[0],
                              sizeof( startMessage ),
                              &bytesWrote,
                              1000 );
      if (nResult != 0)
      {
         DBG( "error %d sending startMessage\n", nResult );
         return nResult;
      }
      if (bytesWrote != sizeof( startMessage ))
      {
         DBG( "invalid write size %d, %lu\n",
              bytesWrote,
              sizeof( startMessage ) );
         return -EIO;
      }
   }

   // Pass to usb_serial_generic_open
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   return usb_serial_generic_open( pPort, pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,31 ))
   return usb_serial_generic_open( pTTY, pPort, pFilp );
#else // > 2.6.31
   return usb_serial_generic_open( pTTY, pPort );
#endif
}

/*===========================================================================
METHOD:
   GobiClose (Free Method)

DESCRIPTION:
   Stop GPS if GPS port, run usb_serial_generic_close

PARAMETERS:
   pTTY    [ I ] - TTY structure (only if kernel > 2.6.26 and <= 2.6.30)
   pPort   [ I ] - USB serial port structure
   pFilp   [ I ] - File structure (only on kernel <= 2.6.30)
===========================================================================*/
#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
void GobiClose(
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
void GobiClose(
   struct tty_struct *        pTTY,
   struct usb_serial_port *   pPort,
   struct file *              pFilp )
#else // > 2.6.30
void GobiClose( struct usb_serial_port * pPort )
#endif
{
   const char stopMessage[] = "$GPS_STOP";
   int nResult;
   int bytesWrote;

   DBG( "\n" );

   // Test parameters
   if ( (pPort == NULL)
   ||   (pPort->serial == NULL)
   ||   (pPort->serial->dev == NULL)
   ||   (pPort->serial->interface == NULL)
   ||   (pPort->serial->interface->cur_altsetting == NULL) )
   {
      DBG( "invalid parameter\n" );
      return;
   }

   // Is this the GPS port?
   if (pPort->serial->interface->cur_altsetting->desc.bInterfaceNumber == 3)
   {
      // Send stopMessage, 1s timeout
      nResult = usb_bulk_msg( pPort->serial->dev,
                              usb_sndbulkpipe( pPort->serial->dev,
                                               pPort->bulk_out_endpointAddress ),
                              (void *)&stopMessage[0],
                              sizeof( stopMessage ),
                              &bytesWrote,
                              1000 );
      if (nResult != 0)
      {
         DBG( "error %d sending stopMessage\n", nResult );
      }
      if (bytesWrote != sizeof( stopMessage ))
      {
         DBG( "invalid write size %d, %lu\n",
              bytesWrote,
              sizeof( stopMessage ) );
      }
   }

   // Pass to usb_serial_generic_close
   if (gpClose == NULL)
   {
      DBG( "NULL gpClose\n" );
      return;
   }

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,26 ))
   gpClose( pPort, pFilp );
#elif (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,30 ))
   gpClose( pTTY, pPort, pFilp );
#else // > 2.6.30
   gpClose( pPort );
#endif
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,25 ))

/*===========================================================================
METHOD:
   GobiReadBulkCallback (Free Method)

DESCRIPTION:
   Read data from USB, push to TTY and user space

PARAMETERS:
   pURB  [ I ] - USB Request Block (urb) that called us

RETURN VALUE:
===========================================================================*/
static void GobiReadBulkCallback( struct urb * pURB )
{
   struct usb_serial_port * pPort = pURB->context;
   struct tty_struct * pTTY = pPort->tty;
   int nResult;
   int nRoom = 0;
   unsigned int pipeEP;

   DBG( "port %d\n", pPort->number );

   if (pURB->status != 0)
   {
      DBG( "nonzero read bulk status received: %d\n", pURB->status );

      return;
   }

   usb_serial_debug_data( debug,
                          &pPort->dev,
                          __FUNCTION__,
                          pURB->actual_length,
                          pURB->transfer_buffer );

   // We do no port throttling

   // Push data to tty layer and user space read function
   if (pTTY != 0 && pURB->actual_length)
   {
      nRoom = tty_buffer_request_room( pTTY, pURB->actual_length );
      DBG( "room size %d %d\n", nRoom, 512 );
      if (nRoom != 0)
      {
         tty_insert_flip_string( pTTY, pURB->transfer_buffer, nRoom );
         tty_flip_buffer_push( pTTY );
      }
   }

   pipeEP = usb_rcvbulkpipe( pPort->serial->dev,
                             pPort->bulk_in_endpointAddress );

   // For continuous reading
   usb_fill_bulk_urb( pPort->read_urb,
                      pPort->serial->dev,
                      pipeEP,
                      pPort->read_urb->transfer_buffer,
                      pPort->read_urb->transfer_buffer_length,
                      GobiReadBulkCallback,
                      pPort );

   nResult = usb_submit_urb( pPort->read_urb, GFP_ATOMIC );
   if (nResult != 0)
   {
      DBG( "failed resubmitting read urb, error %d\n", nResult );
   }
}

#endif

/*===========================================================================
METHOD:
   GobiSuspend (Public Method)

DESCRIPTION:
   Set reset_resume flag

PARAMETERS
   pIntf          [ I ] - Pointer to interface
   powerEvent     [ I ] - Power management event

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent )
{
   struct usb_serial * pDev;

   if (pIntf == 0)
   {
      return -ENOMEM;
   }

   pDev = usb_get_intfdata( pIntf );
   if (pDev == NULL)
   {
      return -ENXIO;
   }

   // Unless this is PM_EVENT_SUSPEND, make sure device gets rescanned
   if ((powerEvent.event & PM_EVENT_SUSPEND) == 0)
   {
      pDev->dev->reset_resume = 1;
   }

   // Run usb_serial's suspend function
   return usb_serial_suspend( pIntf, powerEvent );
}

#if (LINUX_VERSION_CODE <= KERNEL_VERSION( 2,6,23 ))

/*===========================================================================
METHOD:
   GobiResume (Free Method)

DESCRIPTION:
   Restart URBs killed during usb_serial_suspend

   Fixes 2 bugs in 2.6.23 kernel
      1. pSerial->type->resume was NULL and unchecked, caused crash.
      2. set_to_generic_if_null was not run for resume.

PARAMETERS:
   pIntf  [ I ] - Pointer to interface

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int GobiResume( struct usb_interface * pIntf )
{
   struct usb_serial * pSerial = usb_get_intfdata( pIntf );
   struct usb_serial_port * pPort;
   int portIndex, errors, nResult;

   if (pSerial == NULL)
   {
      DBG( "no pSerial\n" );
      return -ENOMEM;
   }
   if (pSerial->type == NULL)
   {
      DBG( "no pSerial->type\n" );
      return ENOMEM;
   }
   if (pSerial->type->resume == NULL)
   {
      // Expected behaviour in 2.6.23, in later kernels this was handled
      // by the usb-serial driver and usb_serial_generic_resume
      errors = 0;
      for (portIndex = 0; portIndex < pSerial->num_ports; portIndex++)
      {
         pPort = pSerial->port[portIndex];
         if (pPort->open_count > 0 && pPort->read_urb != NULL)
         {
            nResult = usb_submit_urb( pPort->read_urb, GFP_NOIO );
            if (nResult < 0)
            {
               // Return first error we see
               DBG( "error %d\n", nResult );
               return nResult;
            }
         }
      }

      // Success
      return 0;
   }

   // Execution would only reach this point if user has
   // patched version of usb-serial driver.
   return usb_serial_resume( pIntf );
}

#endif

/*===========================================================================
METHOD:
   GobiInit (Free Method)

DESCRIPTION:
   Register the driver and device

PARAMETERS:

RETURN VALUE:
   int - negative error code on failure
         zero on success
===========================================================================*/
static int __init GobiInit( void )
{
   int nRetval = 0;
   gpClose = NULL;

   gGobiDevice.num_ports = NUM_BULK_EPS;

   // Registering driver to USB serial core layer
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,4,0 ))
      nRetval = usb_serial_register( &gGobiDevice );
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 ))
      nRetval = usb_serial_register_drivers( gGobiDevices, "Gobi", GobiVIDPIDTable);
#else
      nRetval = usb_serial_register_drivers( &GobiDriver, gGobiDevices);
#endif

   if (nRetval != 0)
   {
      return nRetval;
   }

   // Registering driver to USB core layer
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,4,0 ))
   nRetval = usb_register( &GobiDriver );
   if (nRetval != 0)
   {
      usb_serial_deregister( &gGobiDevice );
      return nRetval;
   }
#endif

   // This will be shown whenever driver is loaded
   printk( KERN_INFO "%s: %s\n", DRIVER_DESC, DRIVER_VERSION );

   return nRetval;
}

/*===========================================================================
METHOD:
   GobiExit (Free Method)

DESCRIPTION:
   Deregister the driver and device

PARAMETERS:

RETURN VALUE:
===========================================================================*/
static void __exit GobiExit( void )
{
   gpClose = NULL;
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 3,4,0 ))
   usb_deregister( &GobiDriver );
   usb_serial_deregister( &gGobiDevice );
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,5,0 ))
   usb_serial_deregister_drivers( gGobiDevices );
#else
   usb_serial_deregister_drivers( &GobiDriver, gGobiDevices );
#endif
}

// Calling kernel module to init our driver
module_init( GobiInit );
module_exit( GobiExit );

MODULE_VERSION( DRIVER_VERSION );
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("Dual BSD/GPL");

module_param( debug, bool, S_IRUGO | S_IWUSR );
MODULE_PARM_DESC( debug, "Debug enabled or not" );
