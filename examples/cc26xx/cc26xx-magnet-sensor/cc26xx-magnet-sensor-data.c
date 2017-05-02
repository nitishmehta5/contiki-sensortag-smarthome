/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-magnet-sensor-data CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *   - The CC2650 LaunchPad
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 *
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Read Magnet Sensor Readings and display message to UART.
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "random.h"
#include "board-peripherals.h"
#include "dev/radio.h"
#include "net/netstack.h"
#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>


/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 2)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF
#define CC26XX_MAGNET_SENSOR		     &reed_relay_sensor
#define CC26XX_RADIO_PKTSIZE			16
#define CC26XX_PAYLOAD_OFFSET			9
/*---------------------------------------------------------------------------*/
static struct etimer et;


/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_magnet_sensor_process, "cc26xx magnet process");
AUTOSTART_PROCESSES(&cc26xx_magnet_sensor_process);
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
static void init_sensors(void)
{

  SENSORS_ACTIVATE(reed_relay_sensor);
}


/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_magnet_sensor_process, ev, data)
{
	int i=0;
	unsigned char radio_pkt[CC26XX_RADIO_PKTSIZE]={0};

  PROCESS_BEGIN();

  printf("CC26XX Sensor Tag as Magnet Sensor\n");

  init_sensors();
  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);

  while(1)
  {
	  PROCESS_YIELD();
	  // Checking whether magnet sensor event occurs.
	  if(ev == sensors_event)
	  {
		  if(data == CC26XX_MAGNET_SENSOR)
		  {
			  if(reed_relay_sensor.value(1) == 1)
				  printf("Door Closed\n");
			  else
				  printf("Door Open\n");
		   if(buzzer_state())
	   	   {
		   	 buzzer_stop();
	   	   }
	   	   else
	   	   {
	   		 buzzer_start(1000);
	   	   }
		  }
	  }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
