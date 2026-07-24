#ifndef PROTOCOL__H_
#define PROTOCOL__H_

#include "sys.h"
#include "stdbool.h"
#include "string.h"

#define FRAME_DATA_MAX_LENGTH   (50)
#define FRAME_USE_CRC32  1   /* Set to 1 for CRC32, 0 for additive checksum */

#define FRAME_HEAD_POS         (0)//֡ͷ�ֽ���һ֡�е�λ��
#define FRAME_FUNCTION_POS     (1)//֡�����ֽ���һ֡�е�λ��
#define FRAME_SEQUENCE_POS     (2)//֡�����ֽ���һ֡�е�λ��
#define FRAME_DATA_LENGTH_POS  (3)//֡���ݳ����ֽ���һ֡�е�λ��
#define FRAME_DATA_POS         (4)//֡�����ֽ���һ֡�е�λ��

/*���ֽ� ֡����ö��*/
typedef enum
{
    UpdateFirmwareStart  =0XC0,//��ʼ�̼���������
    UpdateFirmwareDone   =0XC1,//�̼������������
    BehaviorControl      =0xC2,//��Ϊ��������
    RequestState         =0xC3,//����״̬����
    RouteCoord           =0xC4,//��·��������
}fCommand_e;
/*�����ֽ�ö��*/
typedef enum
{
    ffRespond=0xb0,//��Ӧ
    ffCommand=0xb1,//����
    ffData   =0xb2,//����
    ffIdle   =0xb3,//����
    ffMax,//�����ֽڽ�β
}FrameFunction_e;

/*֡��������ö��*/
typedef enum
{
    FrameFromatErr=0x1,//��ʽ����
    FrameCheckErr =0x2,//У�����
    FrameSeqErr   =0x3,//���д���
    FrameLengthErr=0x4,//���ȴ���
    FrameSuccess  =0x5,//�޴���
}FrameErr_e;

/*֡�ṹ��*/
__packed typedef struct
{
    u8 FrameHead;//֡ͷ �̶�Ϊ0XAA
    FrameFunction_e FrameFunction;//֡���ܱ�ʶ�ֽ�
    u8 FrameSequence;//֡���� 0-255 ѭ����1
    u8 DataLength;//֡���ݳ��� 0-50
    u8 *FrameData;//֡���ݵ�ַ
    u8 FrameChecksum;//֡У��
}Frame_t;

/*֡���ռ�¼*/
__packed typedef struct
{
    Frame_t tFrame;//֡��Ϣ
    u8 DataBuf[FRAME_DATA_MAX_LENGTH];//֡����
    u8 NextFrameSeq;//�´�Ӧ�õ�����
    
    u8 ContinueErrNum;//�����������
    bool IsStartReceive;//�Ƿ�ʼ����
    bool IsFrameNotProcess;//֡���ݻ�δ����
    FrameErr_e FrameErrType;//֡��������
}Rx_FrameRecord;


/*֡���ͼ�¼*/
__packed typedef struct
{
    Frame_t tFrame;//֡��Ϣ
    u8 DataBuf[FRAME_DATA_MAX_LENGTH];//֡����
    u8 NextFrameSeq;//�´�Ӧ�õ�����
    bool IsSendOut;//�Ƿ�����һ֡���ݣ�
    bool IsReceiveRespond;//�Ƿ���ܵ���Ӧ
}Tx_FrameRecord;

extern Rx_FrameRecord FR_Rx;
extern Tx_FrameRecord FR_Tx;

bool bJudgeChecksum(u8 *buf,u8 length);
u8 ucCodeChecksum(u8 *buf,u8 length);

FrameErr_e eFrame_Analy(u8 *buf,u8 length,Rx_FrameRecord *RFR);

u8 ucCode_RespondFrame(Tx_FrameRecord *Tx,Rx_FrameRecord *Rx,u8 *buf);
u8 ucCode_CommandFrame(Tx_FrameRecord *Tx,u8 *Commandbuf,u8 length,u8 *buf);
u8 ucCode_DataFrame(Tx_FrameRecord *Tx,u8 *Databuf,u8 length,u8 *buf);
u8 ucCode_IdleFrame(Tx_FrameRecord *Tx,u8 *buf);

#endif

