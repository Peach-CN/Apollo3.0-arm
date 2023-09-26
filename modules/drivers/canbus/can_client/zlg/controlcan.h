#ifndef THIRD_PARTY_CAN_CARD_LIBRARY_ZLG_CONTROLCAN_H
#define THIRD_PARTY_CAN_CARD_LIBRARY_ZLG_CONTROLCAN_H

#ifdef __cplusplus
extern "C" {
#endif

////文件版本：v2.02 20190609
//接口卡类型定义

enum {
  VCI_USBCAN1 = 3,
  VCI_USBCAN2 = 4,
  VCI_USBCAN2A= 4,
  VCI_USBCAN_E_U = 20,
  VCI_USBCAN_2E_U = 21,
};

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void*
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void*
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

//1.ZLGCAN系列接口卡信息的数据类型。
typedef struct _VCI_BOARD_INFO {
  USHORT hw_Version;
  USHORT fw_Version;
  USHORT dr_Version;
  USHORT in_Version;
  USHORT irq_Num;
  BYTE can_Num;
  CHAR str_Serial_Num[20];
  CHAR str_hw_Type[40];
  USHORT Reserved[4];
} VCI_BOARD_INFO, *PVCI_BOARD_INFO;

//2.定义CAN信息帧的数据类型。
typedef struct _VCI_CAN_OBJ {
  UINT ID;
  UINT TimeStamp;
  BYTE TimeFlag;
  BYTE SendType;
  BYTE RemoteFlag;//是否是远程帧
  BYTE ExternFlag;//是否是扩展帧
  BYTE DataLen;
  BYTE Data[8];
  BYTE Reserved[3];
} VCI_CAN_OBJ, *PVCI_CAN_OBJ;

//3.定义初始化CAN的数据类型
typedef struct _INIT_CONFIG {
 DWORD AccCode;
 DWORD AccMask;
 DWORD Reserved;
 UCHAR Filter;
 UCHAR Timing0;	
 UCHAR Timing1;	
 UCHAR Mode;
} VCI_INIT_CONFIG, *PVCI_INIT_CONFIG;

///////// new add struct for filter /////////
typedef struct _VCI_FILTER_RECORD {
  DWORD ExtFrame;	//是否为扩展帧
  DWORD Start;
  DWORD End;
} VCI_FILTER_RECORD, *PVCI_FILTER_RECORD;

DWORD VCI_OpenDevice(DWORD DeviceType, DWORD DeviceInd, DWORD Reserved);
DWORD VCI_CloseDevice(DWORD DeviceType, DWORD DeviceInd);
DWORD VCI_InitCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_INIT_CONFIG pInitConfig);

DWORD VCI_ReadBoardInfo(DWORD DeviceType, DWORD DeviceInd, PVCI_BOARD_INFO pInfo);

DWORD VCI_SetReference(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, DWORD RefType, PVOID pData);

ULONG VCI_GetReceiveNum(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
DWORD VCI_ClearBuffer(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

DWORD VCI_StartCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);
DWORD VCI_ResetCAN(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd);

ULONG VCI_Transmit(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pSend, UINT Len);
ULONG VCI_Receive(DWORD DeviceType, DWORD DeviceInd, DWORD CANInd, PVCI_CAN_OBJ pReceive, UINT Len, INT WaitTime);

DWORD  VCI_UsbDeviceReset(DWORD DevType, DWORD DevIndex, DWORD Reserved);
DWORD  VCI_FindUsbDevice2(PVCI_BOARD_INFO pInfo);

#ifdef __cplusplus
}
#endif

#endif  // THIRD_PARTY_CAN_CARD_LIBRARY_ZLG_CONTROLCAN_H