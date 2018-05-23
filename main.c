
#include "chuankou.h"
#include "main.h"
#include "JY901.h"
#include "math.h"
#include "duoji.h"
#include "yaogan.h"
#include "ramp.h"
#include "mypid.h"


struct STime		stcTime,stcTime1;
struct SAcc 		stcAcc,stcAcc1;
struct SGyro 		stcGyro,stcGyro1;
struct SAngle 	stcAngle,stcAngle1;
struct SMag 		stcMag,stcMag1;
struct SDStatus stcDStatus,stcDStatus1;
struct SPress 	stcPress,stcPress1;
struct SLonLat 	stcLonLat,stcLonLat1;
struct SGPSV 		stcGPSV,stcGPSV1;
//pitch-90(holeup)-90(holedown),raw,yaw   
float imua[3],imuw[3],imuangle[3],T,lastangle[3],lastheadangle[3],refangle[3],oriangle[3],shangangle[3],gv=0.006,sv=30,headangle[3],kp=8000,yawbias,jumpbias,pidwatch,baund;
int x=0,ready=0,flag,sameyaw=1,stablecnt,direction,shoudongflag,shangflag;
uint32_t pui32ADC0Value[2];
long pwmcalc[2]={1500,1500},tmcount;
int workstate,laststate;

unsigned char insBuffer[6];
 unsigned char insCnt = 0;

ramp Pramp;
ramp Yramp;
int shangflag;
pidregulator pitchpid;
pidregulator yawpid;

//云台电机驱动
// angle=-90,pwmcalc=500
void gyromove(void)
{
	if ((refangle[0]-imuangle[0])>1||(refangle[0]-imuangle[0])<-1)
	{
		;
		pidwatch=pidrealize(&pitchpid,refangle[0]);
		pwmcalc[0]=(pidwatch+90)/180*2000+500;
		VAL_LIMIT(pwmcalc[0],500,2500);
		
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     (uint32_t)pwmcalc[0]);
		

	}
	if ((refangle[2]-imuangle[2])>1||(refangle[2]-imuangle[2])<-1)  //  ori=1500,(-10)-(90)=-100, so ref+100    110-90=20 so ref-20
	{
			pwmcalc[1]=(refangle[2]-oriangle[2]+90)/180*2000+500;
		VAL_LIMIT(pwmcalc[1],500,2500);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                     (uint32_t)pwmcalc[1]);

	}

}
//自动稳定模式
void selfadjust(void)
{
	
//	if(x==0&&imuangle[0]!=0){
//		oriangle[0]=imuangle[0];
//	oriangle[1]=imuangle[1];
//	oriangle[2]=imuangle[2];
//	x++;
//	}
	
	if((oriangle[0]-imuangle[0])>2||(oriangle[0]-imuangle[0])<-2)						//-2~2为死区，防抖
	{
			refangle[0]=refangle[0]+(oriangle[0]-imuangle[0])/kp;
	}
	VAL_LIMIT(refangle[0],-90,90);      //pitch角度限制
	
	if((oriangle[2]-imuangle[2])>5||(oriangle[2]-imuangle[2])<-5)
	{
			refangle[2]=refangle[2]+(oriangle[2]-imuangle[2])/kp;
	}
	
	VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);      //yaw角度限制
	gyromove();
	

}
//头部跟随模式
void follow(void)
{
	
	if((headangle[0]-imuangle[0])>2||(headangle[0]-imuangle[0])<-2)
	{
			refangle[0]=refangle[0]+(headangle[0]+45-imuangle[0])/kp;
	}
	

	
	if((headangle[2]-yawbias-imuangle[2])>2||(headangle[2]-yawbias- imuangle[2])<-2)
	{
			refangle[2]=refangle[2]+(headangle[2]-jumpbias-yawbias-imuangle[2])/kp;
	}
	
	
	VAL_LIMIT(refangle[0],0,90);      //pitch
	VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);     //yaw
	gyromove();
}
//遥控模式
void yaogancontrol(void)
{
		if (pui32ADC0Value[0]<=100)     //pitch move
		{
			refangle[0]-=gv;
			
		}else if(pui32ADC0Value[0]>=3800)
		{
			refangle[0]+=gv;
			
		}
		
		if (pui32ADC0Value[1]<=100)     //yaw move
		{
			refangle[2]-=gv;
			
		}else if(pui32ADC0Value[1]>=3800)
		{
			refangle[2]+=gv;
			
		}
	VAL_LIMIT(refangle[0],0,90);      //pitch
	VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);     //yaw
	gyromove();
}
//每个循环，判断进入哪一种模式，并初始化角度
int getworkstate(void)
{
	//start mode
	if(x==0&&imuangle[0]!=0){
		oriangle[0]=imuangle[0];
	oriangle[1]=imuangle[1];
	oriangle[2]=imuangle[2];
	x++;
	}
	//swich mode need to initialize oriangle（初始角） and refangle（指令角）
	if((ready)&&(workstate!=laststate)&&(imuangle[0]))
	{
		oriangle[0]=imuangle[0];
	oriangle[1]=imuangle[1];
	oriangle[2]=imuangle[2];
//	refangle[0]=imuangle[0];
//	refangle[1]=imuangle[1];
//	refangle[2]=imuangle[2];
		
		x=0;            //
	}
	//state1 need to calibrate
	if((laststate!=1)&&(workstate==1))
	{
			sameyaw=1;   //calibrate yaw
		 ROM_IntEnable(INT_UART3);
		  ROM_IntEnable(INT_UART6);
	}
	
		if((pui32ADC0Value[0]<=100)||(pui32ADC0Value[0]>=3800)||(pui32ADC0Value[1]<=100)||(pui32ADC0Value[1]>=3800))
	{
			shoudongflag=1;
	}else{
			shoudongflag=0;
	}

	laststate=workstate;
	return workstate;
}

//进入特定工作模式
void controltask (void)
{
	switch(getworkstate())
	{
		case 0:{
		selfadjust();
			 ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                             GPIO_PIN_0|GPIO_PIN_1);

			
		}break;
		case 1:{
		follow();
			 ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                             GPIO_PIN_0);

		}break;
		case 2:{
			yaogancontrol();
			 ROM_GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1,
                             GPIO_PIN_1);
			
		}break;
		
		default:break;
	
	}

}
//16进制到10进制
int hextodec(unsigned char insBuf)
{
		int insangle;
	insangle=insBuf/16*10+insBuf%16;
	return insangle; 


}
//处理上位机的指令
void passorders(void)
{
	if(insBuffer[3])
	{
		switch(insBuffer[3])
		{
			case 0x11: refangle[0]+=sv; 	VAL_LIMIT(refangle[0],0,90);       break;
			case 0x13: refangle[0]-=sv; VAL_LIMIT(refangle[0],0,90);       break;	
			case 0x12: refangle[2]+=sv;   VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);   break;  //yaw
			case 0x10: refangle[2]-=sv; VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);  break;    //yaw
			
		
		}
	}
	else
	{
		refangle[0]=hextodec(insBuffer[1]);
		refangle[2]=hextodec(insBuffer[2]);
		VAL_LIMIT(refangle[0],0,90); 
		VAL_LIMIT(refangle[2],oriangle[2]-90,oriangle[2]+90);
	}
	gyromove();
}
//从上位机获取指令
void instructionsget(unsigned char insData)
{
	
		insBuffer[insCnt++]=insData;
	if(insBuffer[0]!=0x81)
	{
		insCnt=0;
		return;
	}
	if (insCnt<6) {return;}
	else
	{
	
		passorders();
		shangflag=1;
		insCnt=0;
	}  

}
//CopeSerialData为串口3中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	

	//USB_TxWrite(&ucData,1);
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
		imua[0]=(float)stcAcc.a[0]/32768*16;
			imua[1]=(float)stcAcc.a[1]/32768*16;
			imua[2]=(float)stcAcc.a[2]/32768*16;
			imuw[0]=(float)stcGyro.w[0]/32768*2000;
			imuw[1]=(float)stcGyro.w[1]/32768*2000;
			imuw[2]=(float)stcGyro.w[2]/32768*2000;
			imuangle[0]=(float)stcAngle.Angle[0]/32768*180;
			imuangle[1]=(float)stcAngle.Angle[1]/32768*180;
			imuangle[2]=(float)stcAngle.Angle[2]/32768*180;
		
		if(imuangle[0]) ready=1;
		
	
	}
}
//头部imu数据处理
void headprocess(unsigned char ucData)
{
	static unsigned char Buffer[250];
	static unsigned char Cnt = 0;	
	

	//USB_TxWrite(&ucData,1);
	Buffer[Cnt++]=ucData;
	if (Buffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		Cnt=0;
		return;
	}
	if (Cnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(Buffer[1])
		{
			case 0x50:	memcpy(&stcTime1,&Buffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"，将接收缓冲区的字符拷贝到数据共同体里面，从而实现数据的解析。
			case 0x51:	memcpy(&stcAcc1,&Buffer[2],8);break;
			case 0x52:	memcpy(&stcGyro1,&Buffer[2],8);break;
			case 0x53:	memcpy(&stcAngle1,&Buffer[2],8);break;
			case 0x54:	memcpy(&stcMag1,&Buffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus1,&Buffer[2],8);break;
			case 0x56:	memcpy(&stcPress1,&Buffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat1,&Buffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV1,&Buffer[2],8);break;
		}
		Cnt=0;
		
			headangle[0]=(float)stcAngle1.Angle[0]/32768*180;
			headangle[1]=(float)stcAngle1.Angle[1]/32768*180;
			headangle[2]=(float)stcAngle1.Angle[2]/32768*180;
		
		
		if((sameyaw==1)&&(imuangle[2]!=0))
		{
			yawbias=headangle[2]-imuangle[2];
			sameyaw++;
		}
	
	}
}

uint32_t g_ui32SysClock;


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

uint32_t yaogandata[2];
int
main(void)
{
   g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 16000000);
	 pwminit();
	
   uartinit();
	
  adcinit();
	pidinit(&pitchpid);
	pidinit(&yawpid);
	SysTickPeriodSet(g_ui32SysClock / 100);
    SysTickIntEnable();
    SysTickEnable();
	ButtonsInit();
	IntEnable(INT_PWM0_0|INT_PWM0_1|INT_UART3);
		IntMasterEnable();
	workstate=2;			//自稳定模式
    while(1)
    {
		controltask();
			
    }
}
