/*
|-------------------------------------------------------------------|
| HCMC University of Technology                                     |
| Telecommunications Departments                                    |
| Wireless Embedded Firmware for Smart Lighting System (SLS)        |
| Version: 2.0                                                      |
| Author: sonvq@hcmut.edu.vn                                        |
| Date: 01/2019                                                     |
| - HW support in ISM band: TelosB, CC2538, CC2530, CC1310, z1      |
| - Support Sensor shield: TSL256x,BMPX8X, Si7021	                |
|-------------------------------------------------------------------|

Topology description:

		|----------|     IPv6     |-----------|	  IPv4/IPv6		|----------|
		| 6LoWPAN  |<------------>|  Gateway  |<--------------->| Client   |   
		| network  |   wireless	  | + BR + DB |  wired/wireless | software |
		|----------|              |-----------|					|----------|

*/


#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/ipv6/uip-ds6-route.h"

//#include <stdlib.h>
//#include <string.h>
//#include "lib/ringbuf.h"

#include "net/ip/uip-debug.h"
#include "dev/leds.h"
#include "net/rpl/rpl.h"
#include "dev/watchdog.h"

#include "random.h"

#include "sys/etimer.h"
#include "sys/ctimer.h"


#ifdef WITH_COMPOWER
#include "powertrace.h"		
#endif

#include "sls.h"	
#include "util.h"	


#ifdef SLS_USING_SKY
#include "dev/uart1.h" 
#endif



#ifdef SLS_USING_CC2538DK
#include "dev/button-sensor.h"
#include "dev/uart.h"
#include "dev/gpio.h"

#ifdef CC2538DK_HAS_SHIELD
#include "dev/i2c.h"
#include "dev/tsl256x.h"
#include "dev/bmpx8x.h"
#include "dev/si7021.h"
#endif

#endif


/*---------------------------------------------------------------------------*/
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_UDP_BUF  ((struct uip_udp_hdr *)&uip_buf[uip_l2_l3_hdr_len])


#define MAX_PAYLOAD_LEN 			120
#define SEND_ASYNC_MSG_CONTINUOUS	TRUE 		// set FALSE to send once
#define SEND_ASYN_MSG_PERIOD		60			// seconds
#define READ_SENSOR_PERIOD			30			// seconds
#define NUM_ASYNC_MSG_RETRANS   	2           // for async msg


// CC2538DK has shield with sensors
#ifdef CC2538DK_HAS_SHIELD
static uint16_t TSL256X_light;
static uint16_t BMPx8x_pressure;
static int16_t 	BMPx8x_temperature;
static uint16_t Si7021_humidity;
static uint16_t Si7021_temperature;
#endif


#ifdef SLS_USING_CC2538DK
static  char rxbuf[MAX_PAYLOAD_LEN];			/* used for UART0 communication */
static 	int cmd_cnt;
#endif


/*---------------------------------------------------------------------------*/
static struct uip_udp_conn *server_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint16_t len, curr_seq, new_seq, async_seq;


/* SLS define */
static 	led_struct_t led_db;
static 	gw_struct_t gw_db;
static 	net_struct_t net_db;
static 	env_struct_t env_db;
static 	cmd_struct_t cmd, reply, emer_reply;
static 	radio_value_t aux;
static	int	state;



/*define timers */
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
static	struct	etimer	et;
static	uint8_t	emergency_status, sent_authen_msg;
static 	uint16_t timer_cnt = 0, timer_cnt_1s = 0;		// use for multiple timer events
static	uint32_t random_delay;


/* define prototype of fucntion call */
static 	void set_connection_address(uip_ipaddr_t *ipaddr);
static 	void get_radio_parameter(void);
static 	void init_default_parameters(void);
static 	void reset_reply_parameters(void);
static 	uint32_t rand_delay();
static	void show_configuration();


#ifdef 	SLS_USING_CC2538DK
static 	unsigned int uart0_send_bytes(const	unsigned  char *s, unsigned int len);
static 	int uart0_input_byte(unsigned char c);
//static 	unsigned int uart1_send_bytes(const	unsigned  char *s, unsigned int len);
//static 	int uart1_input_byte(unsigned char c);
/*sensor define */
#endif 

static 	void send_cmd_to_uart();

static	void process_hello_cmd(cmd_struct_t command);
static	void print_cmd(cmd_struct_t command);
static	void print_cmd_data(cmd_struct_t command);

static 	void send_reply (cmd_struct_t res, uint8_t encryption_en);
static	void blink_led (unsigned char led);
static 	uint8_t is_cmd_of_nw (cmd_struct_t cmd);
static 	uint8_t is_cmd_of_led(cmd_struct_t cmd);
static 	void send_asyn_msg(uint8_t encryption_en);
static 	void get_next_hop_addr();
static 	uint8_t is_connected();
static 	void reset_sequence();


static	uint8_t encryption_phase;
static	uint8_t sent_app_key_ack;


/* if using CC2538DK-SHIELD */
static 	void init_sensor();
static 	void process_sensor(uint8_t verbose);
static 	void set_led_cc2538_shield(int value);




/*---------------------------------------------------------------------------*/
PROCESS(udp_echo_server_process, "SLS server process");
AUTOSTART_PROCESSES(&udp_echo_server_process);

/*---------------------------------------------------------------------------*/
static void init_default_parameters(void) {
	state = STATE_HELLO;
	led_db.id		= LED_ID_MASK;				
	led_db.power	= 120;
	led_db.dim		= 80;
	led_db.status	= STATUS_LED_ON; 
	led_db.temperature = 37;

	gw_db.id		= GW_ID_MASK;				
	gw_db.power		= 150;
	gw_db.status	= GW_CONNECTED; 

	cmd.sfd  = SFD;
	cmd.seq	 = 0;
	cmd.type = MSG_TYPE_REP;
	cmd.len  = sizeof(cmd_struct_t);

	net_db.panid 	= SLS_PAN_ID;
	net_db.connected = FALSE;
	net_db.lost_connection_cnt = 0;
	net_db.authenticated = FALSE;

	emergency_status = DEFAULT_EMERGENCY_STATUS;
	encryption_phase = FALSE;
	sent_authen_msg = FALSE;
	sent_app_key_ack = FALSE;

	curr_seq = 0;
	new_seq = 0;
	async_seq = 0;

	memset(&env_db, 0,sizeof(env_db));

	// init UART0-1
#ifdef SLS_USING_CC2538DK
	uart_init(0); 		
 	uart_set_input(0,uart0_input_byte);
#endif
}

/*---------------------------------------------------------------------------*/
void print_cmd_data(cmd_struct_t command) {
	uint8_t i;	
  	PRINTF("data = [");
	for (i=0; i<MAX_CMD_DATA_LEN; i++) {PRINTF("%02X",command.arg[i]); }
  	PRINTF("]\n");
}

/*---------------------------------------------------------------------------*/
static void make_packet_for_node(cmd_struct_t *cmd, uint8_t* key, uint8_t encryption_en) {
	if (encryption_en==TRUE) {
    	PRINTF("Key = "); phex_16((uint8_t*)key);
		encrypt_payload(cmd, key);
	} else {
	    PRINTF(" - Encryption:... DISABLED \n");    
	}
}

/*---------------------------------------------------------------------------*/
static void check_packet_for_node(cmd_struct_t *cmd, uint8_t* key, uint8_t encryption_en) {
	if (cmd->sfd != SFD) {
	    PRINTF(" - Maybe received packate is encrypted: SPF = 0x%02X \n",cmd->sfd);   
		if (encryption_en==TRUE)
			decrypt_payload(cmd, key);
		else
	    	PRINTF(" - Decryption:... DISABLED \n");   
	}
	else{
	    PRINTF(" - Received packate is NOT encrypted \n");   
	}
}


/*---------------------------------------------------------------------------*/
static void process_req_cmd(cmd_struct_t cmd){
	uint16_t rssi_sent, i;

	reply = cmd;
	reply.type =  MSG_TYPE_REP;
	reply.err_code = ERR_NORMAL;
	PRINTF("Process REQ ....\n");
	if (state==STATE_NORMAL) {
		switch (cmd.cmd) {
			case CMD_RF_HELLO:
				//leds_on(RED);
				//PRINTF ("Execute CMD = %s\n",SLS_LED_ON);
				break;
			case CMD_RF_AUTHENTICATE:
				break;

			case CMD_RF_LED_ON:
				leds_on(BLUE);
				led_db.status = STATUS_LED_ON;
				PRINTF("Execute CMD = 0x%02X \n",CMD_RF_LED_ON);

				set_led_cc2538_shield(1);
				break;

			case CMD_RF_LED_OFF:
				leds_off(BLUE);
				led_db.status = STATUS_LED_OFF;
				PRINTF("Execute CMD = 0x%02X \n",CMD_RF_LED_ON);

				set_led_cc2538_shield(0);
				break;

			case CMD_RF_LED_DIM:
				leds_toggle(BLUE);
				led_db.status = STATUS_LED_DIM;
				led_db.dim = cmd.arg[0];			
				PRINTF ("Execute CMD = 0x%02X; value = %d\n",CMD_LED_DIM, led_db.dim);
				break;

			case CMD_GET_RF_STATUS:
				reply.arg[0] = led_db.id;
				reply.arg[1] = led_db.power;
				reply.arg[2] = led_db.temperature;
				reply.arg[3] = led_db.dim; 
				reply.arg[4] = led_db.status;
				break;

			/* network commands */				
			case CMD_RF_REBOOT:
				send_reply(reply, encryption_phase);
				clock_delay(50000);
				watchdog_reboot();
				break;

			case CMD_GET_NW_STATUS:
				reply.arg[0] = 0;
				reply.arg[1] = NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE;
				reply.arg[2] = net_db.channel;
				rssi_sent = net_db.rssi + 150;
				PRINTF("rssi_sent = %d \n", rssi_sent);
				reply.arg[3] = (rssi_sent & 0xFF);	
				reply.arg[4] = net_db.lqi;
				reply.arg[5] = net_db.tx_power; 
				reply.arg[6] = (net_db.panid >> 8);
				reply.arg[7] = (net_db.panid) & 0xFF;	
	
				reply.arg[8] = (SECURITY_EN << 4) | NONCORESEC_CONF_SEC_LVL;
				reply.arg[9] = ENCRYPTION_MODE;

				// add next hop: only last 8 bytes, because first 8 bytes are FE80::0
				for (i=0; i<8; i++) {
					reply.arg[10+i] = net_db.next_hop[8+i];
				}

				break;

			case CMD_GET_GW_STATUS:
				break;

			case CMD_GET_APP_KEY:
				memcpy(&reply.arg,&net_db.app_code,16);
				break;

			case CMD_RF_REPAIR_ROUTE:
				rpl_repair_root(RPL_DEFAULT_INSTANCE);
				break;

			default:
				reply.err_code = ERR_UNKNOWN_CMD;			
		}
	} 
	else if (state==STATE_HELLO) {
		reply = cmd;	
		reply.err_code = ERR_IN_HELLO_STATE;
	}
	
}

/*---------------------------------------------------------------------------*/
static void process_hello_cmd(cmd_struct_t command){
	uint16_t rssi_sent, i;	
	uint32_t tem;

	get_radio_parameter();
	reply = command;
	reply.type =  MSG_TYPE_HELLO;
	reply.err_code = ERR_NORMAL;

	if (state==STATE_HELLO) {
		switch (command.cmd) {

			case CMD_RF_HELLO:
				leds_off(RED);
				break;

			case CMD_RF_AUTHENTICATE: 
				tem = (command.arg[0] << 8) | command.arg[1];
				net_db.challenge_code = tem & 0xFFFF;
				net_db.challenge_code_res = hash(net_db.challenge_code);
				PRINTF("challenge_code = 0x%04X \n", net_db.challenge_code);
				PRINTF("challenge_res  = 0x%04X \n", net_db.challenge_code_res);

				reply.arg[0] = (net_db.challenge_code_res >> 8 ) & 0xFF;
				reply.arg[1] = (net_db.challenge_code_res) & 0xFF;

				reply.arg[2] = net_db.channel;
				rssi_sent = net_db.rssi + 150;
				PRINTF("rssi_sent = %d \n", rssi_sent);
				reply.arg[3] = (rssi_sent & 0xFF);	
				reply.arg[4] = net_db.lqi;
				reply.arg[5] = net_db.tx_power; 
				reply.arg[6] = (net_db.panid >> 8);
				reply.arg[7] = (net_db.panid) & 0xFF;	
	
				reply.arg[8] = SECURITY_EN;
				reply.arg[8] = (reply.arg[8] << 4) | NONCORESEC_CONF_SEC_LVL;
				reply.arg[9] = ENCRYPTION_MODE;

				// add next hop: only last 8 bytes, because first 8 bytes are FE80::0
				for (i=0; i<8; i++) {
					reply.arg[10+i] = net_db.next_hop[8+i];
				}

				sent_authen_msg = TRUE;
				reset_sequence();
				net_db.authenticated = FALSE;
				encryption_phase = FALSE;				

				leds_off(GREEN);
				break;

			case CMD_SET_APP_KEY:
				state = STATE_NORMAL;
				memcpy(&net_db.app_code,&cmd.arg,16);
				net_db.authenticated = TRUE;
				PRINTF("Got the APP_KEY: authenticated \n");
			    PRINTF("Key = [");
    			for (i=0; i<=15; i++) {	PRINTF("%02X ", net_db.app_code[i]);}
    			PRINTF("]\n");
    		
				encryption_phase = net_db.authenticated;
				sent_app_key_ack = TRUE;
				PRINTF("encryption_phase =  %d: \n", encryption_phase);				

				env_db.id = reply.arg[16];
				PRINTF("My APP-ID = %d \n", env_db.id);

				leds_on(GREEN);
				break;

			default:
				reply.err_code = ERR_IN_HELLO_STATE;
				break;
		}	

		
	} else { // state!=STATE_HELLO
		switch (command.cmd) {
			case CMD_RF_HELLO:
				break;

			case CMD_RF_AUTHENTICATE: 
				tem = (command.arg[0] << 8) | command.arg[1];
				net_db.challenge_code = tem & 0xFFFF;
				net_db.challenge_code_res = hash(net_db.challenge_code);
				PRINTF("challenge_code = 0x%04X \n", net_db.challenge_code);
				PRINTF("challenge_res  = 0x%04X \n", net_db.challenge_code_res);

				reply.arg[0] = (net_db.challenge_code_res >> 8 ) & 0xFF;
				reply.arg[1] = (net_db.challenge_code_res) & 0xFF;

				reply.arg[2] = net_db.channel;
				rssi_sent = net_db.rssi + 150;
				PRINTF("rssi_sent = %d\n", rssi_sent);
				reply.arg[3] = (rssi_sent & 0xFF);			
				reply.arg[4] = net_db.lqi;
				reply.arg[5] = net_db.tx_power; 
				reply.arg[6] = (net_db.panid >> 8);
				reply.arg[7] = (net_db.panid) & 0xFF;	

				reply.arg[8] = (SECURITY_EN << 4) | NONCORESEC_CONF_SEC_LVL;
				reply.arg[9] = ENCRYPTION_MODE;

				// add next hop: only last 8 bytes, because first 8 bytes are fe80::0
				for (i=0; i<8; i++) {
					reply.arg[10+i] = net_db.next_hop[8+i];
				}

				sent_authen_msg = TRUE;
				reset_sequence();
				encryption_phase = FALSE;
				net_db.authenticated = FALSE;
				
				leds_off(GREEN);
				leds_on(GREEN);
				break;

			case CMD_SET_APP_KEY:
				state = STATE_NORMAL;
				memcpy(&net_db.app_code,&cmd.arg,16);
				net_db.authenticated = TRUE;
				encryption_phase = net_db.authenticated;

				PRINTF("Got the APP_KEY: authenticated \n");
			    PRINTF("Key = [");
    			for (i=0; i<=15; i++) {PRINTF("%02X", net_db.app_code[i]);}
    			PRINTF("]\n");

				sent_app_key_ack = TRUE;
				PRINTF("encryption_phase =  %d: \n", encryption_phase);		

				env_db.id = reply.arg[16];
				PRINTF("My APP-ID = %d \n", env_db.id);		

				leds_on(GREEN);
				break;				
		}
	}
}


/*---------------------------------------------------------------------------*/
static uint8_t is_cmd_of_nw (cmd_struct_t cmd) {
	return  (cmd.cmd==CMD_GET_RF_STATUS) ||
			(cmd.cmd==CMD_GET_NW_STATUS) ||
			(cmd.cmd==CMD_RF_HELLO) ||
			(cmd.cmd==CMD_RF_LED_ON) ||
			(cmd.cmd==CMD_RF_LED_OFF) ||
			(cmd.cmd==CMD_RF_LED_DIM) ||			
			(cmd.cmd==CMD_RF_TIMER_ON) ||			
			(cmd.cmd==CMD_RF_TIMER_OFF) ||			
			(cmd.cmd==CMD_SET_APP_KEY) ||		
			(cmd.cmd==CMD_GET_APP_KEY) ||	
			(cmd.cmd==CMD_RF_REBOOT) ||		
			(cmd.cmd==CMD_RF_REPAIR_ROUTE) ||
			(cmd.cmd==CMD_RF_AUTHENTICATE);		
}


/*----------------------------------------------------------------------*/
static uint8_t is_cmd_of_led (cmd_struct_t cmd) {
	return !is_cmd_of_nw(cmd);
}


/*----------------------------------------------------------------------*/
void print_cmd(cmd_struct_t cmd) {
	PRINTF("Rx CMD-struct: sfd=0x%02X; len=%d; seq=%d; type=0x%02X; cmd=0x%02X; err_code=0x%04X\n",
							cmd.sfd, cmd.len, cmd.seq, cmd.type, cmd.cmd, cmd.err_code);
}

/*----------------------------------------------------------------------*/
static void tcpip_handler(void)	{

	memset(buf, 0, MAX_PAYLOAD_LEN);
  	if(uip_newdata()) {
  		blink_led(GREEN);
    	len = uip_datalen();
    	memcpy(buf, uip_appdata, len);
    	PRINTF("In state = %d, received a packet from [", state);
    	PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
    	PRINTF("]:%u, %d bytes of data: ", UIP_HTONS(UIP_UDP_BUF->srcport), len);
		
    	uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    	server_conn->rport = UIP_UDP_BUF->srcport;

		get_radio_parameter();
		reset_reply_parameters();
		
		//p = &buf;	cmdPtr = (cmd_struct_t *)(&buf);
		cmd = *(cmd_struct_t *)(&buf);

		// data decryption
		check_packet_for_node(&cmd, net_db.app_code, encryption_phase);	

		print_cmd(cmd);
		print_cmd_data(cmd);

		/* check CRC of command: no need to check, lower layer will do*/
		if (check_crc_for_cmd(&cmd)==TRUE) {PRINTF("Good CRC \n"); }
		else {PRINTF("Bad CRC \n");}

		reply = cmd;		

		//process command 
		new_seq = cmd.seq;
		PRINTF("[new_seq/old_seq] = [%d/%d] \n", new_seq, curr_seq);			

		if (is_cmd_of_nw(cmd)) {
			/* get a REQ */
			if (cmd.type==MSG_TYPE_REQ) {
				if ((cmd.cmd == CMD_RF_AUTHENTICATE) || (cmd.cmd == CMD_SET_APP_KEY)) {
					/* do not check sequence */
					process_req_cmd(cmd);
				} else if (new_seq > curr_seq) {	// if not duplicate packet
					process_req_cmd(cmd);
					curr_seq = new_seq;	
				}	
				
			/* get a HELLO, do not check sequence */
			} else if (cmd.type==MSG_TYPE_HELLO) { 
				process_hello_cmd(cmd);	
			
			} 
			PRINTF("Reply for NW command: \n");
			send_reply(reply, encryption_phase);
		}	

		/* LED command , send command to LED-driver */
		if (is_cmd_of_led(cmd)){
			if (state==STATE_NORMAL) {
#if defined(SLS_USING_SKY) || defined(SLS_USING_Z1)			/* used for Cooja simulate the reply from LED driver */
				PRINTF("Reply for LED-driver command: \n");
				send_reply(reply, encryption_phase);
#endif
		send_cmd_to_uart();
			}
		}	
  	}
	return;
}


/*---------------------------------------------------------------------------*/
static void blink_led(unsigned char led) {
	int i;
	for (i=0; i<3; i++) { 
		leds_toggle(led);
		//clock_delay_usec((uint16_t)10000000);
		clock_delay((uint16_t)500000);
		leds_toggle(led);
		//clock_delay_usec((uint16_t)10000000);
		clock_delay((uint16_t)500000);
	}	
}


/*---------------------------------------------------------------------------*/
#ifdef SLS_USING_CC2538DK
static int uart0_input_byte(unsigned char c) {
	int i;
	if (c==SFD) {
		cmd_cnt=1;
		rxbuf[cmd_cnt-1]=c;
	}
	else {
		cmd_cnt++;
		rxbuf[cmd_cnt-1] = c;
		if (cmd_cnt == 10) {
			for (i=0; i<=8; i++) {
				emer_reply.arg[i] = rxbuf[i+1];
			}

			send_asyn_msg(encryption_phase);
		}
		
		//if (cmd_cnt==sizeof(cmd_struct_t)) {		/* got the full reply */
		//	cmd_cnt = 0;
		//	emer_reply = *((cmd_struct_t *)(&rxbuf));
		//	PRINTF("Get cmd from LED-driver %s \n",rxbuf);
			
			/* processing emergency reply */
		//	if (emer_reply.type == MSG_TYPE_ASYNC) {
		//		emergency_status = TRUE;
		//		send_asyn_msg(encryption_phase);
		//	} 
		//	else if (emer_reply.type == MSG_TYPE_REP) {		//send reply
		//		reply = emer_reply;
		//		send_reply(reply, encryption_phase);		/* got a Reply from LED-driver, send to orginal node */
				//blink_led(GREEN);
		//	}
		//}		
	}
	return 1;
}


/*---------------------------------------------------------------------------*/
static unsigned int uart0_send_bytes(const	unsigned  char *s, unsigned int len) {
	unsigned int i;
	for (i = 0; i<len; i++) {
		uart_write_byte(0, (uint8_t) (*(s+i)));
   	}   
   return 1;
}
#endif



/*---------------------------------------------------------------------------*/
static void send_cmd_to_uart() {
#ifdef SLS_USING_CC2538DK
	uart0_send_bytes((const unsigned  char *)(&cmd), sizeof(cmd));	
#endif
}

/*---------------------------------------------------------------------------*/
static void reset_reply_parameters(void) {
	memset(&reply, 0, sizeof(reply));
}

/*---------------------------------------------------------------------------*/
static void get_radio_parameter(void) {
#ifndef SLS_USING_CC2530DK
	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &aux);
	net_db.channel = (unsigned int) aux;
	PRINTF("CH = %u, ", (unsigned int) aux);	

 	aux = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	net_db.rssi = (int8_t)aux;
 	PRINTF("RSSI = %d dBm, ", net_db.rssi);

	aux = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);
	net_db.lqi = aux;
 	PRINTF("LQI = %u, ", aux);

	NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &aux);
	net_db.tx_power = aux;
 	PRINTF("Tx Power = %d dBm\n", aux);
#endif 	
}



/*---------------------------------------------------------------------------*/
static void set_connection_address(uip_ipaddr_t *ipaddr) {
 	// change this IP address depending on the node that runs the server: [aaaa::1]
 	uip_ip6addr(ipaddr, 0xaaaa,0x0000,0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001);
}


/*---------------------------------------------------------------------------*/
static void send_reply(cmd_struct_t res, uint8_t encryption_en) {
	cmd_struct_t response;

	response = res;
	gen_crc_for_cmd(&response);
	make_packet_for_node(&response, net_db.app_code, encryption_en);

	/* echo back to sender */	
	PRINTF("Reply to [");
	PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
	PRINTF("]:%u %u bytes\n", UIP_HTONS(UIP_UDP_BUF->srcport), sizeof(res));
	uip_udp_packet_send(server_conn, &response, sizeof(response));

	/* Restore server connection to allow data from any node */
	uip_create_unspecified(&server_conn->ripaddr);
	//memset(&server_conn->ripaddr, 0, sizeof(server_conn->ripaddr));
	//server_conn->rport = 0;
	blink_led(GREEN);
}

/*---------------------------------------------------------------------------*/
static void reset_sequence(){
	async_seq 	= 0;
	curr_seq 	= 0;
	new_seq 	= 0;
}



/*---------------------------------------------------------------------------*/
static void send_asyn_msg(uint8_t encryption_en){ 
	cmd_struct_t response;

	// pass data of env_db to payload	
	memcpy(&emer_reply.arg, &env_db,sizeof(env_db));

	//attach rssi

	// no retransmission here
	if (is_connected()==TRUE) {
		if ((net_db.authenticated == TRUE) || (emer_reply.cmd == ASYNC_MSG_JOINED)) {		// if authenticated or request Authen
			async_seq++;
			emer_reply.sfd = SFD;
			emer_reply.type = MSG_TYPE_ASYNC;
			emer_reply.err_code = ERR_NORMAL;
			emer_reply.seq = async_seq;
		
			response = emer_reply;
			gen_crc_for_cmd(&response);
			make_packet_for_node(&response, net_db.app_code, encryption_en);

			/* debug only*/	
			PRINTF("Client sending (%d bytes) ASYNC msg [%d], CMD = 0x%02X, to BR [", sizeof(response), async_seq, emer_reply.cmd);
			PRINT6ADDR(&client_conn->ripaddr);
			PRINTF("] \n");

			clock_delay(rand_delay());
			uip_udp_packet_send(client_conn, &response, sizeof(response));	
		}
		else {
			PRINTF("Failed to send ASYNC msg [%d]: Route to BR found but unauthenticated...\n", async_seq);
		}
	}
	else {
		PRINTF("Failed to send ASYNC msg: No route to BR found...\n");
	}
}


/*---------------------------------------------------------------------------*/
static uint32_t rand_delay() {
	return (uint32_t)( ((random_rand() / env_db.id) + (random_rand() % env_db.id)*100))*env_db.id;
}


/*---------------------------------------------------------------------------*/
static void et_timeout_hanler(){
	uint8_t j = 0;

	timer_cnt_1s++;

	if (timer_cnt_1s<10) {
		timer_cnt_1s ++;

		// heart beat of network connectivity
		// RED blink: connected; RED solid: disconnected
		if (is_connected()==TRUE) {leds_toggle(RED);}
		else {leds_on(RED);}
	}
	else {
		timer_cnt_1s = 0; 	//reset 1s cnt
		timer_cnt++;		//10s, 20s, 30s,...

		// count in 600s: timer_cnt timeout = 10s
		if (timer_cnt==60) {timer_cnt =0;}

		/* read sensors every READ_SENSOR_PERIOD = 10s */
		if ((timer_cnt % (READ_SENSOR_PERIOD / 10))==0) {
			PRINTF("\nTimer: %ds expired... reading sensors, timer_cnt (10s) = %d \n", READ_SENSOR_PERIOD, timer_cnt);
    		process_sensor(PRINT_SENSOR);

    		if (timer_cnt > 60) {  // crash or something wrong--> reboot
    			PRINTF("- Something wrong...RESET");
    			watchdog_reboot();	
    		}
    	}	
	
		/* send an async msg every SEND_ASYN_MSG_PERIOD seconds */
		if ((timer_cnt % (SEND_ASYN_MSG_PERIOD / 10) )==0) {				
			if ((state==STATE_NORMAL) && (emergency_status==TRUE) && (net_db.authenticated==TRUE)) {	
				PRINTF("\nTimer: %ds expired: send an async msg, retrans_max = %d \n", SEND_ASYN_MSG_PERIOD, NUM_ASYNC_MSG_RETRANS);
				process_sensor(PRINT_SENSOR);		//read sensor if having shield

				emer_reply.cmd = ASYNC_MSG_SENT;
				emer_reply.err_code = ERR_NORMAL;

				// try to send NUM_ASYNC_MSG_RETRANS times
				j =NUM_ASYNC_MSG_RETRANS;
				while ((j--)>0) {
					random_delay = rand_delay();
					PRINTF("Delay a random time = %u ms for retrans_num = %d \n", (uint16_t)((uint32_t)(random_delay*2.83)/1000), NUM_ASYNC_MSG_RETRANS-j-1);
					clock_delay(random_delay);
					send_asyn_msg(encryption_phase);
					async_seq--; // send multiple msg with same seq
					//j++;
				} 
				async_seq++;	
				emergency_status = SEND_ASYNC_MSG_CONTINUOUS;		// send once or continuously, if FALSE: send once.
				blink_led(GREEN);
			}
		}

		/* 50s events */
		if ((timer_cnt % 5)==0) {
			/* check join/disjoin in 50s */
			if (is_connected()==TRUE) {
	    		//PRINTF("Network status: JOINED\n");
				get_next_hop_addr();
    			net_db.connected = TRUE;
	    		net_db.lost_connection_cnt = 0;
	    		if ((net_db.authenticated==FALSE)  && (sent_authen_msg==FALSE)){
					PRINTF("Send authentication request: \n");					
					reset_sequence();
					emer_reply.cmd = ASYNC_MSG_JOINED;
					emer_reply.err_code = ERR_NORMAL;
					clock_delay(rand_delay());
					send_asyn_msg(encryption_phase);

	    			leds_off(GREEN);
	    		}

    		} else { // not connected
	    		PRINTF("Network status: NOT CONNECTED \n");
		    	net_db.lost_connection_cnt++; 	

		    	// if lost connection in 150s then confirm connected = FALSE, but still authenticated
	    		if (net_db.lost_connection_cnt==3) {	
	    			net_db.connected = FALSE;
	    			net_db.lost_connection_cnt=0;
					PRINTF("Lost parent DAG in 150s... try to repair root\n");
					rpl_repair_root(RPL_DEFAULT_INSTANCE);

					// reset authentication 		
		    		net_db.authenticated= FALSE;
		    		sent_authen_msg = FALSE;
	    		}
    		}
    	}
    }
}	


/*---------------------------------------------------------------------------*/
static uint8_t is_connected() {
    rpl_dag_t *dag = rpl_get_any_dag();
    return (dag && dag->instance->def_route);
}

/*---------------------------------------------------------------------------*/
static void get_next_hop_addr(){
#if (UIP_CONF_IPV6_RPL)
	//int i;
    rpl_dag_t *dag = rpl_get_any_dag();
    if(dag && dag->instance->def_route) {
	    memcpy(&net_db.next_hop, &dag->instance->def_route->ipaddr, sizeof(uip_ipaddr_t));
	    //PRINTF("Next_hop addr [%d] = ", sizeof(uip_ipaddr_t));
	    //for (i=0; i<sizeof(uip_ipaddr_t);i++) {PRINTF("0x%02X ", net_db.next_hop[i]);}
	    //PRINTF("\n");
    } 
#endif        
}


/*---------------------------------------------------------------------------*/
static void init_sensor() {
#ifdef CC2538DK_HAS_SHIELD
	GPIO_SET_OUTPUT(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));	
	GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 | 0x01<<3 | 0x01<<4 | 0x01<<5));
	SENSORS_ACTIVATE(bmpx8x);
	if(TSL256X_REF == TSL2561_SENSOR_REF) {
    	PRINTF("Light sensor test --> TSL2561\n");
  	} else if(TSL256X_REF == TSL2563_SENSOR_REF) {
    	PRINTF("Light sensor test --> TSL2563\n");
  	} else {
    	PRINTF("Unknown light sensor reference, aborting\n");
  	}

	SENSORS_ACTIVATE(tsl256x);
  	//TSL256X_REGISTER_INT(light_interrupt_callback);
	tsl256x.configure(TSL256X_INT_OVER, 0x15B8);
#endif	
}

/*---------------------------------------------------------------------------*/
static void set_led_cc2538_shield(int value){
// turn on/off 3 LEDs
#ifdef CC2538DK_HAS_SHIELD
	if (value>0) 	{	GPIO_SET_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 ));} 
	else 			{	GPIO_CLR_PIN(GPIO_B_BASE, (0x01 | 0x01<<1 | 0x01<<2 ));}
#endif	
}


/*---------------------------------------------------------------------------*/
static void process_sensor(uint8_t verbose) {
#ifdef CC2538DK_HAS_SHIELD
	int32_t 	tData;
	uint32_t 	rhData;
	uint8_t 	H, L;

	BMPx8x_pressure = bmpx8x.value(BMPx8x_READ_PRESSURE);
    BMPx8x_temperature = bmpx8x.value(BMPx8x_READ_TEMP);
    TSL256X_light = tsl256x.value(TSL256X_VAL_READ);
    	

   	if(TSL256X_light != TSL256X_ERROR) {
      	env_db.light = TSL256X_light;
	     //context-aware control here
		set_led_cc2538_shield(env_db.light < 20);		

    } else {
    	PRINTF("Error, enable the DEBUG flag in the tsl256x driver for info, or check if the sensor is properly connected \n");
    }	

	if((BMPx8x_pressure != BMPx8x_ERROR) && (BMPx8x_temperature != BMPx8x_ERROR)) {
     	//PRINTF("BMPx8x : Pressure = %u.%u(hPa), \n", pressure / 10, pressure % 10);
    	//PRINTF("Temperature = %d.%u(ºC) \n", temperature / 10, temperature % 10);
    	env_db.pressure = BMPx8x_pressure;
    	env_db.temp = BMPx8x_temperature;
    } else {
    	PRINTF("Error, enable the DEBUG flag in the BMPx8x driver for info, or check if the sensor is properly connected \n");
    }
		
		
	//convert Si7021 temperature
	Si7021_temperature = si7021_readTemp(TEMP_NOHOLD);	
	H = (uint8_t)(Si7021_temperature >> 8);
	L = (uint8_t)(Si7021_temperature & 0xFF);
  	tData = ((uint32_t)H  << 8) + (L & 0xFC);
	tData = (((tData) * 21965L) >> 13) - 46850;

		//convert Si7021 humidity 
	Si7021_humidity = si7021_readHumd(RH_NOHOLD);
	H = (uint8_t)(Si7021_humidity >> 8);
	L = (uint8_t)(Si7021_humidity & 0xFF);
	rhData = ((uint32_t)H << 8) + (L & 0xFC);
	rhData = (((rhData) * 15625L) >> 13) - 6000;
	env_db.humidity = Si7021_humidity;
		
	if (verbose) {
		PRINTF("\n--------------------- READING SENSORS ----------------------\n");
	    PRINTF(" - Temperature (Si7021) = %d.%d (ºC) \n", (uint16_t)(tData /1000), (uint16_t)(tData % 1000));
    	PRINTF(" - Humidity (Si7021)    = %d.%d (RH) \n", (uint16_t)(rhData/1000), (uint16_t)(rhData % 1000));
    	PRINTF(" - Temperature (BMPx8x) = %d.%d (ºC) \n", (uint16_t)(env_db.temp/10), (uint16_t)(env_db.temp % 10));
	    PRINTF(" - Pressure (BMPx8x)    = %d.%d (hPa)\n", (uint16_t)(env_db.pressure / 10), (uint16_t)(env_db.pressure % 10));
    	PRINTF(" - Light (TSL256X)      = %d (lux)\n", (uint16_t)(env_db.light));
		PRINTF("------------------------------------------------------------\n");
	}
#else
	// paste dump data
	env_db.temp = 375;
	env_db.light = 405;
	env_db.pressure = 750;
	env_db.humidity = 9700;	
	if (verbose) {
		PRINTF("\n--------------------- READING DUMP SENSORS -----------------\n");
    	PRINTF("T = %d (ºC); L = %d (lux); P = %d (hPa); H = %d (RH) \n", 
    		(uint16_t)(env_db.temp), (uint16_t)(env_db.light), (uint16_t)(env_db.pressure),(uint16_t)(env_db.humidity) );
		PRINTF("------------------------------------------------------------\n");
	}
#endif	

}


/*---------------------------------------------------------------------------*/
void show_configuration() {
	PRINTF("--------------------- NODE INFORMATION-------------\n");
	PRINTF("- PAN ID = 0x%04x; chanel = %d; with PA = %d \n", SLS_PAN_ID, RF_CHANNEL, CC2538DK_WITH_CC2592);
	PRINTF("- Security enable =%d, LLSEC level = %d, Encryption mode = %d \n", SECURITY_EN, NONCORESEC_CONF_SEC_LVL, ENCRYPTION_MODE);
	PRINTF("- Routing: WITH_NON_STORING = %d\n", WITH_NON_STORING);	
	PRINTF("- Channel check rate = %d\n", NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE);	
#ifdef CC2538DK_HAS_SHIELD		
	PRINTF("- Init parameters, timers, sensor_shield = ENABLED, reading_sensor_interval = %d \n", READ_SENSOR_PERIOD);
#else
	PRINTF("- Init parameters, timers, sensor_shield = DISABLED, reading_sensor_interval = %d  \n", READ_SENSOR_PERIOD);
#endif
	
	PRINTF("- Async msg sending interval = %d, num_of_retrans_asyn = %d \n", SEND_ASYN_MSG_PERIOD, NUM_ASYNC_MSG_RETRANS);	
	PRINTF("---------------------------------------------------\n");
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_echo_server_process, ev, data) {
  	/* Variables inside a thread should be declared as static */

	PROCESS_BEGIN();
 
  	//NETSTACK_MAC.off(1); 		/* disable RDC */
	show_configuration();	

	init_default_parameters();
	
	/* timer for events */
	etimer_set(&et, CLOCK_SECOND*1);

	/*if having sensor shield */
	init_sensor();
	
	/* read sensor for the first time */
	process_sensor(PRINT_SENSOR);

	/* setup server connection for querry */
	server_conn = udp_new(NULL, UIP_HTONS(0), NULL);
  	if (server_conn == NULL) 	{PROCESS_EXIT();}
  	udp_bind(server_conn, UIP_HTONS(SLS_NORMAL_PORT));
	
  	/* setup client connection for asyn message to server [aaaa::1] */
  	set_connection_address(&server_ipaddr);
	client_conn = udp_new(&server_ipaddr, UIP_HTONS(SLS_EMERGENCY_PORT), NULL);
	PRINTF("Set up connection to server [aaaa::1] \n");	


	/* print power trace */
/*
#if (WITH_COMPOWER==1)
  	powertrace_sniff(POWERTRACE_ON);
#endif
*/  	

#if (WITH_COMPOWER==1)
	PRINTF("Init power trace = 10s...\n");		
	powertrace_start(CLOCK_SECOND *10);
#endif

	PRINTF("Enter forever loop...\n");	
 	while(1) {
    	PROCESS_YIELD();

    	/* get a network packet */
    	if(ev == tcpip_event) {
    		get_next_hop_addr();
      		tcpip_handler();
    	} 	
    	/* ev timeout */
    	else if (ev==PROCESS_EVENT_TIMER) {
    		et_timeout_hanler();
    		etimer_restart(&et);
   		}
  	}
	PROCESS_END();
}