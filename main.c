/*
    Nodo RS485 ST03
*/     

#include <stdio.h>
#include <string.h>
#include "thread.h"
#include "shell.h"
#include "xtimer.h"
#include <inttypes.h>

#include "ztimer.h"
#include "board.h"

#include "periph/i2c.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "periph/wdt.h"

//************************************
// Indirizzo della scheda da 1 a 255
//************************************

unsigned char myaddress=27;

#define RED_LED                 GPIO_PIN(PA, 7)
#define GREEN_LED               GPIO_PIN(PA, 8)
#define RS485_DE                GPIO_PIN(PA, 6) 

/* Harvest-10 */
#define HDC_ENABLE              GPIO_PIN(PA, 27)

#define HDC3020_ADDR            (0x44)
#define HDC3020_MEAS_DELAY      (12000)

#define RED_LED_ON              gpio_set(RED_LED);
#define RED_LED_OFF             gpio_clear(RED_LED);

#define GREEN_LED_ON            gpio_set(GREEN_LED);
#define GREEN_LED_OFF           gpio_clear(GREEN_LED);

#define RS485_DE_ON             gpio_set(RS485_DE);
#define RS485_DE_OFF            gpio_clear(RS485_DE);

#define BUFFER_MAX_LEN 20

// Stati possibili durante la ricezione del pacchetto SNAP

#define SNAP_NOSTATE 0
#define SNAP_HDB2    1     
#define SNAP_HDB1    2
#define SNAP_DAB1    3
#define SNAP_SAB1    4
#define SNAP_DB1     5
#define SNAP_CRC2    6
#define SNAP_CRC1    7

int buffer_pointer=0;
unsigned char rxbuffer[BUFFER_MAX_LEN];
unsigned char txbuffer[BUFFER_MAX_LEN];

double temp;
double hum;

int current_state=SNAP_NOSTATE;
int next_state=SNAP_NOSTATE;

// Calcolo del  CRC16

void crc16(unsigned short *crc, char c) {
	int i,j;

	for (i=0; i!=8; c>>=1, i++) {
		j = (c^(*crc)) & 1;
		(*crc)>>=1;

		if (j) {
			(*crc)^=0xa001;
		}
	}
}

// Callback richiamata alla ricezione di ogni carattere dalla RS485

static void rx_cb(void *arg, uint8_t inByte) {
        
    // Questa riga serve solo per non far apparire un messaggio di
    // errore in compilazione    
    if (arg==NULL) {};
	
    if (current_state==SNAP_CRC1) {
        return;
    }

    if (inByte==0x51 && current_state==SNAP_NOSTATE) {
		current_state=SNAP_HDB2;
		buffer_pointer=0;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
    }

	if (inByte==0x41 && current_state==SNAP_HDB2) {
		current_state=SNAP_HDB1;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
	}

   if (inByte==myaddress && current_state==SNAP_HDB1) {
      current_state=SNAP_DAB1;
      rxbuffer[buffer_pointer]=inByte;
      buffer_pointer++;
      return;
    }

	if (inByte==0x00 && current_state==SNAP_DAB1) {
		current_state=SNAP_SAB1;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
	}

	if (inByte==0x02 && current_state==SNAP_SAB1) {
		current_state=SNAP_DB1;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
    }

	if (current_state==SNAP_DB1) {
		current_state=SNAP_CRC2;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
    }

	if (current_state==SNAP_CRC2) {
		current_state=SNAP_CRC1;
		rxbuffer[buffer_pointer]=inByte;
		buffer_pointer++;
		return;
    }

    current_state=SNAP_NOSTATE;
    buffer_pointer=0;
}    


// Lettura temperatura e umidita' dal sensore

int read_hdc(double *temp, double *hum) {
    int status = 0, retry = 10;
    uint8_t command[2] = {0x24, 0x00};
    uint8_t data[6];

    ztimer_sleep(ZTIMER_USEC, 4000);
    if (i2c_write_bytes(I2C_DEV(0), HDC3020_ADDR, command, sizeof(command), 0)) {
        puts("ERROR: starting measure\r\n");
        return 1;
    }
    ztimer_sleep(ZTIMER_USEC, HDC3020_MEAS_DELAY);
    do {
        status = i2c_read_bytes(I2C_DEV(0), HDC3020_ADDR, data, sizeof(data), 0);
        if (status) {
            retry--;
            if (retry < 0) {
              puts("ERROR: reading data\r\n");
              return 1;
            }
            ztimer_sleep(ZTIMER_USEC, 100);
        }
    } while(status);

    if (temp) {
        *temp = ((data[0] << 8) + data[1]) * 175. / (1 << 16) - 45;
        //printf("Temp %.2f\r\n",*temp);
    }

    if (hum) {
        *hum =  ((data[3] << 8) + data[4]) * 100. / (1 << 16);
        //printf("Hum %.2f\r\n",*hum);
    }
    return 0;
}

int main(void) {
	int dev=1;
	int res;
	unsigned char sht75_data[4];
    unsigned short hexdata16bit;

	//char line_buf[SHELL_DEFAULT_BUFSIZE];
    //shell_run(NULL, line_buf, SHELL_DEFAULT_BUFSIZE);

    gpio_init(HDC_ENABLE, GPIO_OUT);
    gpio_set(HDC_ENABLE);

    gpio_init(RS485_DE, GPIO_OUT);
    gpio_init(RED_LED, GPIO_OUT);
    gpio_init(GREEN_LED, GPIO_OUT);
    RS485_DE_OFF
    RED_LED_OFF
    GREEN_LED_OFF

    //xtimer_msleep(2000U);

	for (int i=0;i<2;i++) {
		RED_LED_ON
		xtimer_msleep(500U);
		RED_LED_OFF
		xtimer_msleep(500U);
	}

	for (int i=0;i<2;i++) {
		GREEN_LED_ON
		xtimer_msleep(500U);
		GREEN_LED_OFF
		xtimer_msleep(500U);
	}

    printf("\r\n");
    printf("ST03 board 1.8 - Addr=%d (0x%02X)\r\n",myaddress,myaddress);

    i2c_acquire(I2C_DEV(0));

	/* initialize UART */
    res = uart_init(UART_DEV(dev), 19200, rx_cb, (void *)dev);
    if (res == UART_NOBAUD) {
        printf("Error: Given baudrate not possible\r\n");
        return 1;
    }
    else if (res != UART_OK) {
        puts("Error: Unable to initialize UART device\r\n");
        return 1;
    }

	if (uart_mode(UART_DEV(dev), 8, UART_PARITY_NONE, 1) != UART_OK) {
        printf("Error: Unable to apply UART settings\r\n");
		xtimer_msleep(1000U);
        return 1;
    }


    int i=0;
    int check_sensor_counter=0;
    int blinking_green_led_tick=0;
    int valid_data=false;
    unsigned short crc;
    
	// Set the whatchdog at 10 seconds
    
    
    wdt_setup_reboot(0, 10000);
    wdt_start();
    
    
    // Loop principale
    for(;;) {

		blinking_green_led_tick++;
		if (blinking_green_led_tick>=0 && blinking_green_led_tick<10) {
			wdt_kick();
			GREEN_LED_ON
		} else {	
			GREEN_LED_OFF
		} 	
		if (blinking_green_led_tick>=20) {
			blinking_green_led_tick=0;
		} 	

		check_sensor_counter++;
		if (check_sensor_counter==100) {
			check_sensor_counter=0;
			if (read_hdc(&temp, &hum)==0) {
				GREEN_LED_ON
				valid_data=true;
				printf("Temp:%.2f Hum:%.2f\r\n",temp,hum);
				GREEN_LED_OFF
			} else {
				valid_data=false;
				printf("Bad data\r\n");
			}
		}	
		xtimer_msleep(100U);
	
        // Se e' stato ricevuto un pacchetto SNAP ne controlla il CRC

        if (current_state==SNAP_CRC1) {

            // Controlla la validità del CRC

            crc=0;
            for (buffer_pointer=0;buffer_pointer<5;buffer_pointer++) {
               crc16(&crc,rxbuffer[buffer_pointer]);
            }
            if ( ((crc>>8)&0x00FF)==rxbuffer[5] && ((crc)&0x00FF)==rxbuffer[6] ) {
                printf("Pacchetto OK !\r\n");
            } else {
                printf("Pacchetto CRC ERRATO !\r\n");
            }

            i++;
            printf("Pacchetto n: %04d : ",i);
            for (buffer_pointer=0;buffer_pointer<7;buffer_pointer++) {
              printf("%02X ",rxbuffer[buffer_pointer]);
            }
            printf("\r\n");
        	xtimer_msleep(100U);

             if (valid_data==true) {
				if (rxbuffer[2]==myaddress) {
					RED_LED_ON
					printf("Pacchetto per me !\r\n");
                    //hum=70;
					//printf("Umidita [%02f]\r\n] ",hum);

                    // Converte il valore di temperatura e umidita letto
                    // in valori binari nel formato simile a quello generato
                    // dal sensore SHT75 usato sul Termoigrometro a PIC


                    // Temperatura
                    hexdata16bit=(unsigned short)((temp+40)/0.01);
                    sht75_data[0]=(hexdata16bit>>8)&0xFF;
                    sht75_data[1]=(hexdata16bit)&0xFF;

                    // Umidita
                    hexdata16bit=(unsigned short)(hum*30);
                    sht75_data[2]=(hexdata16bit>>8)&0xFF;
                    sht75_data[3]=(hexdata16bit)&0xFF;

					// Invia il pacchetto SNAP di risposta
					
					txbuffer[0]=0x54; 		//SYNC
					txbuffer[1]=0x50; 		//HDB2
					txbuffer[2]=0x44; 		//HDB1
					txbuffer[3]=0x00; 		//DAB1
					txbuffer[4]=myaddress;	//SAB1
					txbuffer[5]=sht75_data[0];	//DB1
					txbuffer[6]=sht75_data[1];	//DB2
					txbuffer[7]=sht75_data[2];	//DB3
					txbuffer[8]=sht75_data[3];	//DB4
				

					// Calcola il CRC16
					// Controlla la validità del CRC

					crc=0;
					for (buffer_pointer=1;buffer_pointer<9;buffer_pointer++) {
						printf("[%02X] ",txbuffer[buffer_pointer]);
						crc16(&crc,txbuffer[buffer_pointer]);
					}
					printf("[%04X]\r\n",crc);
					txbuffer[9]=(crc>>8)&0x00FF;	//CRC2
					txbuffer[10]=crc&0x00FF; 		//CRC1	
					
					RS485_DE_ON
					uart_write(UART_DEV(dev), (uint8_t *)txbuffer,11);
					RS485_DE_OFF
					RED_LED_OFF
				}
           }


            buffer_pointer=0;
            current_state=SNAP_NOSTATE;
        }
    }
    printf("Bye, bye...\n");
	xtimer_msleep(2000U);
    return 0;
}


//(void) printf("%d\r\n",__LINE__);
/* https://doc.riot-os.org/group__drivers__periph__uart.html */
