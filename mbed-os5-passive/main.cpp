
#include "mbed.h"
#include <math.h>
#define pi 3.14159265358979323846f
#include "MTi2.h"
Serial pc(USBTX,USBRX);
Ticker timer1;
//UART for Bluetooth
RawSerial serial1(PA_9,PA_10,115200);
void Rx_interrupt();
int ITflag=0;
char myRxData[6]={0};
//IMU
MTi2Class IMU(1000000,PB_15,PB_14,PB_13,PB_1);//Create MTi2 Class (Frequency, MOSI, MISO, SCLK, CS)
float pitch,roll,yaw;
//encoder的code
SPI spi(D4, D5, D3); // mosi, miso, sclk
DigitalOut encoder_cs(D9); //SPI_CS1
bool button_state = false;
unsigned short read_flag = 0;
unsigned short data_1 = 0;
unsigned short data_2 = 0;
unsigned short data_3 = 0;
unsigned short data_4 = 0;
unsigned short data_5 = 0;
unsigned short data_6 = 0;
unsigned short data_7 = 0;
unsigned short data_8 = 0;
unsigned short data_9 = 0;
unsigned short data_10 = 0;
unsigned short data_11 = 0;
unsigned short data_12 = 0;
unsigned short data_13 = 0;
unsigned short data_14 = 0;
unsigned short data_15 = 0;


unsigned short total = 0;

unsigned short angle = 0;
unsigned short angle2 = 0;
unsigned short angle3 = 0;
unsigned short angle3_1 = 0;
unsigned short angle3_2 = 0;
unsigned short angle4 = 0;
unsigned short angle4_1 = 0;
unsigned short angle4_2 = 0;
unsigned short angle5 = 0;
unsigned short angle5_1 = 0;
unsigned short angle5_2 = 0;
unsigned short angle6 = 0;
unsigned short angle7 = 0;
unsigned short angle7_1 = 0;
unsigned short angle8 = 0;
unsigned short angle8_1 = 0;
unsigned short angle8_2 = 0;
unsigned short angle9 = 0;
unsigned short angle9_1 = 0;
unsigned short angle9_2 = 0;
unsigned short angle10 = 0;
unsigned short angle10_1 = 0;
unsigned short angle10_2 = 0;
unsigned short angle11 = 0;
unsigned short angle11_1 = 0;
unsigned short angle11_2 = 0;
unsigned short angle12 = 0;
unsigned short angle12_1 = 0;
unsigned short angle12_2 = 0;
// orientation part
unsigned short O_ang1 = 0;
unsigned short O_ang2 = 0;
unsigned short O_len = 0;
unsigned short O_ang1_temp = 0;
unsigned short O_ang2_temp = 0;
unsigned short O_len_temp = 0;


void init_IO();
void init_TIMER();
void timer1_ITR();
void init_UART();
void read_position();
void UART_TX();
void init_SPI();

// main() runs in its own thread in the OS
int main()
{
    pc.baud(115200);
    
    button_state = false;
    
    
    wait_ms(1000);
    init_IO();
    init_TIMER();
    init_SPI();
    init_UART();
    IMU.MTi2_Init();
    while (true) {
        
        if(read_flag){
            
            read_position();
            UART_TX();
        }
        
    }
}

void init_IO()
{
    encoder_cs = 1;
}
void init_TIMER()
{
    timer1.attach_us(&timer1_ITR, 50000.0); // the address of the function to be attached (timer1_ITR) and the interval ``1000 micro-seconds)
}
void init_SPI()
{
    spi.format(16,3);
    spi.frequency(1000000);  // 1MHz clock rate
}
void init_UART()
{
    pc.baud(115200);  // baud rate:115200
}
void Rx_interrupt(){
    ITflag=1;
    
    for(int i=0;i<6;i++){
        myRxData[i]=serial1.getc();
    }
    
}
void timer1_ITR(){
    read_flag = 1 ;
}
void read_position(){
    
    encoder_cs = 0;  // Select the device by seting chip select low
    wait_us(100);

    data_1 = spi.write(0x00);
    data_2 = spi.write(0x00);
    data_3 = spi.write(0x00);
    data_4 = spi.write(0x00);
    data_5 = spi.write(0x00);
    data_6 = spi.write(0x00);
    data_7 = spi.write(0x00);
    data_8 = spi.write(0x00);
    data_9 = spi.write(0x00);
    data_10 = spi.write(0x00);
    data_11 = spi.write(0x00);
    data_12 = spi.write(0x00);
    data_13 = spi.write(0x00);
    data_14 = spi.write(0x00);
    //data_15 = spi.write(0x00);
    
    encoder_cs = 1;  // Deselect the device
    angle = (data_1 >> 3);
    
    angle2 = (data_2 & 0xFFF);
    
    angle3_1 = (data_3 & 0x1FF);
    angle3_1 = (angle3_1 << 3);
    angle3_2 = (data_4 >> 13);
    angle3 = angle3_1 + angle3_2;
    
    angle4_1 = (data_4 << 10);
    angle4_1 = (angle4_1 >> 4);
    angle4_2 = (data_5 >> 10);
    angle4 = angle4_1 + angle4_2;
    
    angle5_1 = (data_5 << 13);
    angle5_1 = (angle5_1 >> 4);
    angle5_2 = (data_6 >> 7);
    angle5 = angle5_1 + angle5_2;
    
    angle6 = (data_7 >> 4);
    
    angle7_1 = (data_8 >> 1);
    angle7 = (angle7_1 & 0xFFF);
    
    angle8_1 = (data_9 << 6);
    angle8_1 = (angle8_1 >> 4) ;
    angle8_2 = (data_10 >> 14);
    angle8 = angle8_1 + angle8_2;
    
    angle9_1 = (data_10 << 9);
    angle9_1 = (angle9_1 >> 4);
    angle9_2 = (data_11 >> 11);
    angle9 = angle9_1 + angle9_2;
    
    angle10_1 = (data_11 & 0xF); 
    angle10_1 = (angle10_1 << 8);
    angle10_2 = (data_12 >> 8);
    angle10 = angle10_1 + angle10_2; 
    
    angle11_1 = (data_12 & 0x1);
    angle11_1 = (angle11_1 << 11);
    angle11_2 = (data_13 >> 5);
    angle11 = angle11_1 + angle11_2; 
    
    angle12_1 = (data_14 << 2);
    angle12_1 = (angle12_1 >> 4);
    angle12 = angle12_1 ;
    

    
    read_flag = 0 ;
}

void UART_TX()
{
    
    IMU.ReadData();
    wait_ms(1);
    //init_position = 2600 - init_position;
    //init_position = (int16_t)(init_position+1024-IMU.euler[0]*90/7.91f);//
    //O_ang1_temp=myRxData[1]<<8;
    //O_ang2_temp=myRxData[3]<<8;
    //O_len_temp=myRxData[5]<<8;
    //O_ang1=myRxData[0]+O_ang1_temp;
    //O_ang2=myRxData[2]+O_ang2_temp;
    //O_len=myRxData[4]+O_len_temp;
//    pc.printf("%.5f,\n\r%.5f,\n\r%.2f,\n\r%.2f,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%.3f, %.3f, %.3f\n\r",v_L,v_H,ss,init_position,angle,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,angle10,angle11, pitch, roll, yaw);
//pc.printf("r%d,\n\r",init_position);
///    pc.printf("%.5f,\n\r%.5f,\n\r%.2f,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r",v_L,v_H,ss,init_position,angle,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,angle10,angle11);//common used
pc.printf("%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%d,\n\r%.2f,\n\r",O_ang1,O_ang2,O_len,angle,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,angle10,angle11,angle12,IMU.euler[0]);//COE used
//pc.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",myRxData[0],myRxData[1],myRxData[2],myRxData[3],myRxData[4],myRxData[5],myRxData[6],myRxData[7],myRxData[8],myRxData[9],myRxData[10],myRxData[11],myRxData[12]);
//    wait_us(100000);
//    T[4] = sp2.c[0];
//    T[5] = sp2.c[1];
//pc.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n\r",O_ang1,O_ang2,O_len,init_position,angle,angle2,angle3,angle4,angle5,angle6,angle7,angle8,angle9,angle10,angle11,IMU.euler[0]);

}