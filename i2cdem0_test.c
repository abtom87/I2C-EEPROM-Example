#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define FOSC            16000000UL
#define DEV_ADDR        0x50
#define baud_rate       9600
#define baud            FOSC/16/baud_rate-1
#define ACK             1
#define NACK            0



void usart_init(unsigned int b_rate)
{
    UBRR0H=0x00;
    UBRR0L=0x00;
    UCSR0B=0x00;
    UCSR0C=0x00;
    UDR0  =0x00;

    UBRR0H  =(unsigned char)b_rate>>8;
    UBRR0L  =(unsigned char)b_rate;

    
    UCSR0B = ( (1<<RXEN0)  | (1<<TXEN0) );
    UCSR0C = ( (1<<UCSZ00) | (1 << UCSZ01));
    
    
}

void Transmit(unsigned char c)
{
   while( !(UCSR0A & (1<<UDRE0)) );
     UDR0=c;
}





typedef enum status_codes_t{ START=0x08, REP_START=0x10,  MT_SLA_ACK=0x18, MT_SLA_NACK=0x20, MT_SLA_DATA_ACK = 0x28, MR_SLA_ACK=0x40, MR_SLA_NACK=0x48, MR_SLA_DATA_ACK=0x50, MR_SLA_DATA_NACK=0x58 }statuscode;

typedef enum result_t{FAIL, SUCCESS}result;

result check_TWI_status( statuscode stat )
{
   switch(stat)
   {
      case START:       if ((TWSR & 0xF8) == START )
     			  return SUCCESS;
   		        else
     			  return FAIL;
     		        break;
      
      case REP_START:     if ((TWSR & 0xF8) == REP_START )
     			    return SUCCESS;
   		          else
     			    return FAIL;
     		          break;
      
      
      case MT_SLA_ACK:    if ((TWSR & 0xF8) == MT_SLA_ACK )
     			    return SUCCESS;
   		          else
     			    return FAIL;
     		          break;
      
      case MT_SLA_NACK:    if ((TWSR & 0xF8) == MT_SLA_NACK )
     			    return SUCCESS;
   		           else
     			    return FAIL;
     		           break;
      
      
      case MT_SLA_DATA_ACK:    if ((TWSR & 0xF8) == MT_SLA_DATA_ACK )
     			        return SUCCESS;
   		               else
     			        return FAIL;
     		               break;
     		               
       case MR_SLA_ACK:       if ((TWSR & 0xF8) == MR_SLA_ACK )
     			        return SUCCESS;
   		              else
     			        return FAIL;
     		              break;
     		               
     		               
      
      case MR_SLA_NACK:       if ((TWSR & 0xF8) == MR_SLA_NACK )
     			        return SUCCESS;
   		              else
     			        return FAIL;
     		              break;
     
     
      case MR_SLA_DATA_ACK:    if ((TWSR & 0xF8) == MR_SLA_DATA_ACK )
     			         return SUCCESS;
   		               else
     			         return FAIL;
     		               break;
      
      
      case MR_SLA_DATA_NACK:    if ((TWSR & 0xF8) == MR_SLA_DATA_NACK )
     			          return SUCCESS;
   		                else
     			          return FAIL;
     		                break;
     		                
   	
   
     default: return FAIL; break;
   
   }
   
}

void Debug_LED_ON()
{
   PORTD|=1<<PD4; //Turn on LED on PIN4 of PORTD
}

void set_clock(const unsigned int freq_in_khz)
{
   uint32_t twi_freq=0 ;
   const unsigned int prescaler=4;
   
   TWCR = 1<<TWEN;     //Enable TWI module
   TWSR |=(1<<TWPS0); //Prescaler set to 4
   
   
   twi_freq = freq_in_khz* 1000;

   TWBR  = (FOSC/twi_freq - 16)/(2*prescaler) ;   

}

void send_start()
{
   TWCR= ( (1<<TWINT) | (1<<TWEN) | (1<<TWSTA) ); //send START
   while( !(TWCR & (1<<TWINT)) ) ;                //wait for TWINT flag SET
}

void send_stop()
{
   TWCR= ( (1<<TWINT) | (1<<TWEN) | (1<<TWSTO) ); //send STOP
}

void send_address(char addr_w)
{

   TWDR=addr_w ; //7bit  address + W bit (write bit)
   TWCR = (1<<TWINT) | (1<<TWEN); //Clear TWINT bit to start transmission of address
   while (!(TWCR &(1<<TWINT)));  //wait for TWINT flag SET
   
}


void  transmit_data(char data)
{
   TWDR=data;
   TWCR = (1<<TWINT) | (1<<TWEN); //Clear TWINT bit to start transmission of Data on the bus
   while (!(TWCR &(1<<TWINT)));  //wait for TWINT flag SET

}

//Enter Master Tx Mode
void enter_MT_mode(char dev_address, uint16_t eeprom_addr)
{
   char UpperByte_Addr=0, LowerByte_Addr=0;
   uint8_t i=0;
   
   UpperByte_Addr = (uint8_t)(eeprom_addr>>8);
   LowerByte_Addr = (uint8_t)eeprom_addr;
   
   send_start();
   if(check_TWI_status(START)==FAIL)
     Debug_LED_ON();
   
   
   send_address(dev_address<<1);
   if(check_TWI_status(MT_SLA_ACK)==FAIL)
     Debug_LED_ON();
   
   transmit_data(UpperByte_Addr);   
   if(check_TWI_status(MT_SLA_DATA_ACK)==FAIL)
     Debug_LED_ON();
   
   
   transmit_data(LowerByte_Addr);   
   if(check_TWI_status(MT_SLA_DATA_ACK)==FAIL)
     Debug_LED_ON();
     
     //send 16 Bytes
    for(i=0;i<16;i++)
    {
       transmit_data(0xFA+i);
       if(check_TWI_status(MT_SLA_DATA_ACK)==FAIL)
         Debug_LED_ON();
        _delay_ms(1);
  
    }
    send_stop();

}
uint8_t read_data_nack()
{
   uint8_t data=0;
   TWCR = (1<<TWINT)| (1<<TWEN) ; //Clear TWINT bit to start transmission of Data on the bus
   while (!(TWCR &(1<<TWINT)));  //wait for TWINT flag SET
   data=TWDR;
   return data;

}

uint8_t read_data_ack()
{
   uint8_t data=0;
   
   
   TWCR = (1<<TWINT)| (1<<TWEN)  | (1<<TWEA); //Clear TWINT bit to start transmission of Data on the bus
   while (!(TWCR &(1<<TWINT)));  //wait for TWINT flag SET
   data=TWDR;
   return data;

}


char read_from_slave_dev(char dev_address, uint16_t addr_to_read )
{

   uint8_t recvd_data=0, UpperByte=0, LowerByte=0;
   
   UpperByte = (uint8_t) (addr_to_read>>8);
   LowerByte = (uint8_t) (addr_to_read);
   
   //Step 1 REP Start
   send_start();  
   
   //Step 2. check Repeated Start code from Slave
   if(check_TWI_status(REP_START)==FAIL)
    Debug_LED_ON();
  
   //Step 3. Send Address + /W & check for ACK
   send_address(dev_address << 1);
   if(check_TWI_status(MT_SLA_ACK)==FAIL)
    Debug_LED_ON();
   
   // Step 4. Send UpperByte of Address in the EEPROM chip
   transmit_data(UpperByte);   
   // Step 6. Check for ACK from Slave
   if(check_TWI_status(MT_SLA_DATA_ACK)==FAIL)
    Debug_LED_ON();
   
    // Step 7. Send LowerByte of Address in the EEPROM chip
   transmit_data(LowerByte); 
   // Step 8. Check for ACK from Slave
   if(check_TWI_status(MT_SLA_DATA_ACK)==FAIL)
    Debug_LED_ON();
 
}

//Master Rx mode
char read_page(char dev_address, uint8_t response_type)
{
  
   char recvd_data;

   if(response_type==ACK)
      recvd_data=read_data_ack();
   else
      recvd_data=read_data_nack();

   return recvd_data;

}
int main (void)
{
   DDRC=0xff;
   PORTC=0x30;   //Enable internal pullups on PORTC PINS  SDA(PC4) ,SCL(PC5)
   DDRD=0xFF;   //Port D as output
   char byte_to_tx[16];
   int i=0;
   
   usart_init(baud);
   
   set_clock(100); //setting clock to 100KHz
 
   //Enter Master Tx mode and pass the address to be written
   enter_MT_mode(DEV_ADDR,0x0100);
   
   
   _delay_ms(5); 
   
   read_from_slave_dev(DEV_ADDR,0x0100);   
   //Step 1. Send start
   send_start();
   //Step 2. check Repeated Start code from Slave
   if(check_TWI_status(REP_START)==FAIL)
    Debug_LED_ON();
    
   //Step 3. //Shift 0x50 by 1 bit + send Read bit(0x01)
   send_address( (char)(DEV_ADDR<<1)| 0x01 ); 
   // Step 4. Check for MasterRx ACK from Slave
   if(check_TWI_status(MR_SLA_ACK)==FAIL)
    Debug_LED_ON();
   
   
    for(i=0;i<16;i++)
    {   
        _delay_ms(1);
        if(i==15)
           byte_to_tx[i]=read_page(DEV_ADDR,NACK);
        else
           byte_to_tx[i]=read_page(DEV_ADDR,ACK);
    }
    send_stop();
    
    for(i=0;i<16;i++)
    {
         _delay_ms(1000);
         Transmit(byte_to_tx[i]);
    }
       
   return 0;
}

