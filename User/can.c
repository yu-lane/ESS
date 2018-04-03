#include "can.h" 
#include "include.h" 
extern int can_addr;
extern CanTxMsg TxMessage;
extern CanTxMsg RxMessage;
/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*外设时钟设置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* Configure CAN pin: RX PA11*/									          
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	/* Configure CAN pin: TX PA12 */									               
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	 	/************************CAN通信参数设置**********************************/
	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理 
    CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  自动唤醒模式
    CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
    CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
    CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常发送模式
    CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+6+3)/4=0.8Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(u32 Receive_ID,u32 ID_Mask)
{
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)Receive_ID<<3)&0xFFFF0000)>>16;				//要过滤的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)Receive_ID<<3)|CAN_ID_STD|CAN_RTR_DATA)&0xFFFF; //要过滤的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= (ID_Mask&0xffff0000)>>16;			//过滤器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= ID_Mask&0x0000ffff;			//过滤器低16位每位必须匹配

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_NVIC_Config();
  CAN_Mode_Config();
  CAN_Filter_Config(can_addr,0x00000000);   
}


/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	 
void CAN_Send_Std_Msg(int can_addr,unsigned char *data,char datanum)
{	  
	unsigned char ii=0;
  TxMessage.StdId=can_addr-0x100;	
	//TxMessage.ExtId=0x1314;
	TxMessage.DLC=datanum;							 //数据长度为2字节
	TxMessage.IDE=CAN_ID_STD;					 //扩展模式
  TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
	for(ii=0;ii<datanum;ii++)
	{
		TxMessage.Data[ii] = data[ii];
	}
	
	CAN_Transmit(CAN1, &TxMessage);

}



/*************************END OF FILE******************************/


