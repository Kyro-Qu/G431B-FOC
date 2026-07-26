#ifndef   _VOFA_H_
#define   _VOFA_H_

/*vofa��ͨ������*/
#define VOFA_CH_COUNT               12

/*���ͽṹ��*/
typedef struct{
    float fdata[VOFA_CH_COUNT];
    const unsigned char tail[4];          /*β֡ 0x00, 0x00, 0x80, 0x7f*/ 
}VOFA_Send_Handle_t;

void VOFA_Task(void);



#endif
