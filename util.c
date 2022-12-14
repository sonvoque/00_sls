/*
|-------------------------------------------------------------------|
| HCMC University of Technology                                     |
| Telecommunications Departments                                    |
| Wireless Embedded Firmware for Smart Lighting System (SLS)        |
| Version: 2.0                                                      |
| Author: sonvq@hcmut.edu.vn                                        |
| Date: 01/2019                                                     |
| HW support in ISM band: TelosB, CC2538, CC2530, CC1310, z1        |
|-------------------------------------------------------------------|*/

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "net/ip/uip-debug.h"
#include "sls.h"
#include "util.h"


#define CBC 1
#define ECB 1
#include "aes_lib.h" 

/*---------------------------------------------------------------------------*/
uint16_t gen_crc16(uint8_t *data_p, unsigned short  length) {
    unsigned char i;
    unsigned int data;
    unsigned int crc = 0xffff;
    uint8_t len;
    len = length;

    if (len== 0)
        return (~crc);
    do    {
        for (i=0, data=(unsigned int)0xff & *data_p++; i < 8; i++, data >>= 1) {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
            else  crc >>= 1;
        }
    } while (--len);

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);
    return (crc);
}

/*---------------------------------------------------------------------------*/
void gen_crc_for_cmd(cmd_struct_t *cmd) {
    uint16_t crc16_check;
    uint8_t byte_arr[MAX_CMD_LEN-2];
    memcpy(&byte_arr, cmd, MAX_CMD_LEN-2);
    crc16_check = gen_crc16(byte_arr, MAX_CMD_LEN-2);
    cmd->crc = (uint16_t)crc16_check;
    PRINTF(" - Generate CRC16 [0x%04X]... done \n", crc16_check);
}

/*---------------------------------------------------------------------------*/
uint8_t check_crc_for_cmd(cmd_struct_t *cmd) {
    uint16_t crc16_check;
    uint8_t byte_arr[MAX_CMD_LEN-2];
    memcpy(&byte_arr, cmd, MAX_CMD_LEN-2);
    crc16_check = gen_crc16(byte_arr, MAX_CMD_LEN-2);
    //PRINTF("CRC-cal = 0x%04X; CRC-val =  0x%04X \n",crc16_check,cmd->crc);
    if (crc16_check == cmd->crc) {
        PRINTF("CRC16...matched\n");
        return TRUE;
    }
    else{
        PRINTF("CRC16 ...failed: CRC-cal = 0x%04X; CRC-val =  0x%04X  \n", crc16_check, cmd->crc);
        return FALSE;        
    }
}


/*---------------------------------------------------------------------------*/
void phex_16(uint8_t* data_16) { // in chuoi hex 16 bytes
    unsigned char i;
    for(i = 0; i < 16; ++i)
        PRINTF("%.2x ", data_16[i]);
    PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
void phex_64(uint8_t* data_64) { // in chuoi hex 64 bytes
    unsigned char i;
    for(i = 0; i < 4; ++i) 
        phex_16(data_64 + (i*16));
    PRINTF("\n");
}


/*---------------------------------------------------------------------------*/
// encrypt 64 bytes ussing AES128 CBC
void encrypt_cbc(uint8_t* data_encrypted, uint8_t* data, uint8_t* key, uint8_t* iv) { 
    //PRINTF("Data input: \n");
    //phex_64(data);
    
    //encrypte 4 rows, each row has 16 bytes
    AES128_CBC_encrypt_buffer(data_encrypted+0, data+0, 16, key, iv);
    AES128_CBC_encrypt_buffer(data_encrypted+16, data+16, 16, key, iv);
    //AES128_CBC_encrypt_buffer(data_encrypted+32, data+32, 16, key, iv);
    //AES128_CBC_encrypt_buffer(data_encrypted+48, data+48, 16, key, iv);

    //PRINTF("\nData encrypted: \n");
    //phex_64(data_encrypted);
}

/*---------------------------------------------------------------------------*/
void  decrypt_cbc(uint8_t* data_decrypted, uint8_t* data_encrypted, uint8_t* key, uint8_t* iv)  {    
    //PRINTF("\nData encrypted: \n");
    //phex_64(data_encrypted);

    AES128_CBC_decrypt_buffer(data_decrypted+0,  data_encrypted+0,  16, key, iv);
    AES128_CBC_decrypt_buffer(data_decrypted+16, data_encrypted+16, 16, key, iv);
    //AES128_CBC_decrypt_buffer(data_decrypted+32, data_encrypted+32, 16, key, iv);
    //AES128_CBC_decrypt_buffer(data_decrypted+48, data_encrypted+48, 16, key, iv);
    
    //PRINTF("Data decrypted: \n");
    //phex_64(data_decrypted);
}


/*---------------------------------------------------------------------------*/
uint16_t hash(uint16_t a) {
	uint32_t tem;
	
    tem =a;
	tem = (a+0x7ed55d16) + (tem<<12);
	tem = (a^0xc761c23c) ^ (tem>>19);
	tem = (a+0x165667b1) + (tem<<5);
	tem = (a+0xd3a2646c) ^ (tem<<9);
	tem = (a+0xfd7046c5) + (tem<<3);
	tem = (a^0xb55a4f09) ^ (tem>>16);
    
    return tem & 0xFFFF;
}


/*---------------------------------------------------------------------------*/
void encrypt_payload(cmd_struct_t *cmd, uint8_t* key) {
    uint8_t payload[MAX_CMD_LEN];    
    
    if (ENCRYPTION_MODE==1){
        scramble_data((uint8_t *)cmd, (uint8_t *)cmd, key);
        PRINTF(" - Scramble data ... done \n");
    }
    else if (ENCRYPTION_MODE==2) {
        memcpy(&payload, cmd, MAX_CMD_LEN);
        encrypt_cbc((uint8_t *)cmd, payload, key, iv);
        PRINTF(" - Encrypt AES128-CBC ... done \n");
    }
}


/*-----------------------0x%02X----------------------------------------------------*/
void decrypt_payload(cmd_struct_t *cmd, uint8_t* key) {
    if (ENCRYPTION_MODE==1) {
        descramble_data((uint8_t *)cmd, (uint8_t *)cmd, key);
        PRINTF(" - Descramble data ... done \n");
    }
    else if (ENCRYPTION_MODE==2) {
        decrypt_cbc((uint8_t *)cmd, (uint8_t *)cmd, key, iv);
        PRINTF(" - Decrypt AES128-CBC ... done \n");
    }
}



//float float_example = 1.11;
//uint8_t bytes[4];
//float2Bytes(float_example, &bytes[0]);
/*---------------------------------------------------------------------------*/
void float2Bytes(float val, uint8_t* bytes_array){
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;

  u.float_variable = val;
  memcpy(bytes_array, u.temp_array, 4);
}

/*---------------------------------------------------------------------------*/
// scramble data with a key
void scramble_data(uint8_t* data_encrypted, uint8_t* data, uint8_t* key) {
    int i;
    for (i=0; i<MAX_CMD_LEN; i++) {
        data_encrypted[i] = data[i] ^ key[i % 4];
    }
} 

/*---------------------------------------------------------------------------*/
// descramble data with a key
void descramble_data(uint8_t* data_decrypted, uint8_t* data_encrypted, uint8_t* key) {
    int i;
    for (i=0; i<MAX_CMD_LEN; i++) {
        data_decrypted[i] = data_encrypted[i] ^ key[i % 4];
    }
} 