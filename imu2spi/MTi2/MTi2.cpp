#include "MTi2.h"

//Constructor & construct cs pin & spi pin
MTi2Class::MTi2Class(SPI& shared_spi, PinName CS):cs_MTi(CS),spi_MTi(shared_spi){
}
//Initialize MTi-2 setting
void MTi2Class::MTi2_Init(){
    cs_MTi = 1;
    spi_MTi.format(8,3);
    spi_MTi.frequency(200000);
    ConfigureProt(1,0,0,0);//Configure DRDY
    
    // 傳送 GoToMeasurement 指令，使 IMU 進入 Measurement Mode 開始輸出資料
    thread_sleep_for(10);
    ctrl_len = 3;
    ctrlBuf[0] = 0x10; // GoToMeasurement
    ctrlBuf[1] = 0x00; // Length
    ctrlBuf[2] = 0xF1; // Checksum
    ControlPipe();
}

void MTi2Class::ControlPipe(){
    cs_MTi = 0;
        SendOpcode(Control);//send opcode
        for(int i = 0;i<ctrl_len;i++){//read data
            buffer[i] = spi_MTi.write(ctrlBuf[i]);
        }
    cs_MTi = 1;
}

void MTi2Class::MeasurementPipe(){
//    printf("MeasurementPipe \r\n");
    cs_MTi = 0;
    SendOpcode(MeasPipe);//send opcode
    for(int i = 0;i<measurementSize;i++){//read data
        buffer[i] = spi_MTi.write(0x00);
    }
    cs_MTi = 1;
}

//Check pipestatus : measure size & nitification size
void MTi2Class::PipeStatus(){
    len = 4;
    cs_MTi = 0;
    SendOpcode(PipeStat);//send opcode
    for(int i = 0;i<len;i++){//read data
        buffer[i] = spi_MTi.write(0x00);
    }
    cs_MTi = 1;
    
    uint16_t n_size = buffer[0] | (buffer[1]<<8);
    uint16_t m_size = buffer[2] | (buffer[3]<<8);
    
    // 安全檢測：如果長度不合理（大於 buffer 長度 120 或是讀到 0xFFFF），強制設為 0 避免記憶體溢位
    if (n_size > 120 || m_size > 120) {
        notificationSize = 0;
        measurementSize = 0;
    } else {
        notificationSize = n_size;
        measurementSize = m_size;
    }
}
//Send op code
void MTi2Class::SendOpcode(uint8_t Opcode)
{
//    printf("SendOpcode \r\n");
    FW[0] = spi_MTi.write((int)Opcode);
    
    for(uint8_t i = 0;i<3;i++){// 3 fillword 
        FW[i+1] = spi_MTi.write((int)i);
    }
}
 
uint8_t MTi2Class::ReadProtInfo(){
    len = 2;
    cs_MTi = 0;
    SendOpcode(ProtInfo);//send opcode
    for(int i = 0;i<len;i++){//read data
        buffer[i] = spi_MTi.write(0x00);
    }
    cs_MTi = 1;
    if(FW[0]!=0xFA||FW[1]!=0xFF||FW[2]!=0xFF||FW[3]!=0xFF){
        printf("Error!!\n");
    }
    return buffer[1];
}
 
void MTi2Class::ConfigureProt(_Bool M,_Bool N,_Bool O,_Bool P)
{
//    printf("ConfigureProt \r\n");
    uint8_t config = (M<<3) | (N<<2) | (O<<1) | (P<<0);
    cs_MTi = 0;
    SendOpcode(ConfigProt);
    spi_MTi.write(config);
    cs_MTi = 1;

}

void MTi2Class::NotificationPipe(){
    if (notificationSize == 0) return;
    cs_MTi = 0;
    SendOpcode(NotiPipe);//send opcode
    for(int i = 0;i<notificationSize;i++){//read data
        buffer[i] = spi_MTi.write(0x00);
    }
    cs_MTi = 1;
}

//Read MTi-2 data
void MTi2Class::ReadData(){
    PipeStatus();//Check pipe statue : measurement size
    wait_us(10);
    
    // 印出讀取到的 PipeStatus 原始位元組與解出的大小，協助除錯
    printf("[SPI-Debug] PipeStatus: %02X %02X %02X %02X | nSize: %d, mSize: %d\n", 
           buffer[0], buffer[1], buffer[2], buffer[3], notificationSize, measurementSize);
           
    if (notificationSize > 0) {
        NotificationPipe();
        wait_us(10);
    }
    
    if (measurementSize == 0) {
        return;
    }
    MeasurementPipe();//Read measurement data
    
    // 印出 MeasurementPipe 讀到的前 16 個位元組
    printf("[SPI-Debug] MeasData (first 16B): ");
    for(int idx = 0; idx < (measurementSize < 16 ? measurementSize : 16); idx++) {
        printf("%02X ", buffer[idx]);
    }
    printf("\n");
    
    if (buffer[0] == 0x36) {
        uint8_t payload_len = buffer[1];
        if (payload_len + 2 <= measurementSize && payload_len + 2 <= 128) {
            uint8_t i = 2;
            while (i + 2 < payload_len + 2) {
                uint16_t xdi = (buffer[i] << 8) | buffer[i+1];
                uint8_t size = buffer[i+2];
                
                if (i + 3 + size > payload_len + 2) {
                    break; // 確保不超出邊界
                }
                
                uint8_t* content = &buffer[i+3];
                
                if (xdi == 0x2030 && size == 12) {        
                    eul[0].data1 = (content[0]<<24) | (content[1]<<16) | (content[2]<<8) | content[3];     
                    eul[1].data1 = (content[4]<<24) | (content[5]<<16) | (content[6]<<8) | content[7]; 
                    eul[2].data1 = (content[8]<<24) | (content[9]<<16) | (content[10]<<8) | content[11]; 
                    euler[0] = eul[0].data2;
                    euler[1] = eul[1].data2;
                    euler[2] = eul[2].data2;
                }
                else if (xdi == 0x4020 && size == 12) {   
                    acc[0].data1 = (content[0]<<24) | (content[1]<<16) | (content[2]<<8) | content[3];     
                    acc[1].data1 = (content[4]<<24) | (content[5]<<16) | (content[6]<<8) | content[7]; 
                    acc[2].data1 = (content[8]<<24) | (content[9]<<16) | (content[10]<<8) | content[11]; 
                    accel[0] = acc[0].data2;
                    accel[1] = acc[1].data2;
                    accel[2] = acc[2].data2;
                }
                else if (xdi == 0x8020 && size == 12) {   
                    gry[0].data1 = (content[0]<<24) | (content[1]<<16) | (content[2]<<8) | content[3];     
                    gry[1].data1 = (content[4]<<24) | (content[5]<<16) | (content[6]<<8) | content[7]; 
                    gry[2].data1 = (content[8]<<24) | (content[9]<<16) | (content[10]<<8) | content[11]; 
                    omega[0] = gry[0].data2;
                    omega[1] = gry[1].data2;
                    omega[2] = gry[2].data2;
                }
                
                i += (3 + size);
            }
        }
    }
}