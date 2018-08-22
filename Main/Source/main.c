/*
 * 受信したらシリアルで表示。ただそれだけ。
 */

// モノスティックでない場合は、下記定義をコメントにする
#define _MONOSTICK

#include <string.h>         // C 標準ライブラリ用
#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"         // シリアル用
#include "sprintf.h"        // SPRINTF 用
#include "ToCoNet_mod_prototype.h" // ToCoNet モジュール定義(無線で使う)

/*
    パソコン側のシリアル通信の設定
    ボーレート      115200
    データ          ８ビット
    パリティ        None(無し)
    ストップビット   1ビット
    フローコントロール None(無し)
*/
#define UART_BAUD 115200 	// シリアルのボーレート
static tsFILE sSerStream;          // シリアル用ストリーム
static tsSerialPortSetup sSerPort; // シリアルポートデスクリプタ

// ToCoNet 用パラメータ。これは通信先のTWELITEと合わせておかないと受信できない！
#define APP_ID   0x67721122
#define CHANNEL  15

// メッセージ出力用
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}

// ハードウェア初期化
static void vInitHardware()
{
	// デバッグ出力用
	vSerialInit();
	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(0);

#ifdef _MONOSTICK
        //LED ON
        vPortAsOutput(16);
        vPortSetHi(16);
#endif
}

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{
    static int count = 0;

    // 起動時メッセージ
    if (eEvent == E_EVENT_START_UP) {
        debug("** Just-Receive-AndShow **");
    }

    // データを受信した
    else if (eEvent == E_ORDER_KICK) {
        count = 5;
#ifdef _MONOSTICK
        //LED ON
        vPortSetLo(16);
#endif
    }

    // 4ms 周期のシステムタイマ通知
    else if (eEvent == E_EVENT_TICK_TIMER) {
        if (count > 0) {
            if(--count == 0) {
#ifdef _MONOSTICK
                //LED OFF
                vPortSetHi(16);
#endif
            }
        }
    }
}

// 割り込み発生後に随時呼び出される
void cbToCoNet_vMain(void)
{
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
}

// パケット受信時
// !本関数の終了(return)を遅延させるべきではありません。
//  通常無線パケットは一般でも最短で 1ms 程度の間隔をあけて伝達されますが、
//  これを超えた長時間の遅延により、無線パケットの取りこぼしが発生するかもしれません。
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
    static uint32 u32SrcAddrPrev = 0;
    static uint8 u8seqPrev = 0xFF;
    static uint32 u32RxTimePrev = 0;

    // 前回と同一の送信元＋シーケンス番号かつ、それが１０秒以内のパケットなら受け流す
    // スリープする機器でRAMをOFFしている場合、シーケンス番号は乱数で生成するので前回と重複する可能性がある
    // それを回避するために１０秒制限という簡易な方法で判別するようにした
    if (pRx->u32SrcAddr == u32SrcAddrPrev && pRx->u8Seq == u8seqPrev &&
        (u32TickCount_ms - u32RxTimePrev) < 10000) {
        return;
    }

    // UARTに出力
    char buf[100];
    int len = (pRx->u8Len < sizeof(buf)) ? pRx->u8Len : sizeof(buf)-1;
    memcpy(buf, pRx->auData, len);
    buf[len] = '\0';

    //送信元S/N シーケンス番号(同じメッセージを重複受信しないための識別番号) 受信したメッセージ
    debug("%08X:%02X %s", pRx->u32SrcAddr, pRx->u8Seq, buf);

    u32SrcAddrPrev = pRx->u32SrcAddr;
    u8seqPrev = pRx->u8Seq;
    u32RxTimePrev = u32TickCount_ms;

    // vProcessEvCore()に受信したことを知らせる
    ToCoNet_Event_Process(E_ORDER_KICK, 0, vProcessEvCore);
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	//static uint8 u8WkState;
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // SPRINTF 初期化
        SPRINTF_vInit128();

        // ToCoNet パラメータ
        //memset (&sToCoNet_AppContext, 0x00, sizeof(sToCoNet_AppContext));
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = TRUE; // アイドル時にも受信
        sToCoNet_AppContext.u8TxMacRetry = 1;

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        // ハードウェア初期化
        vInitHardware();

        // MAC 層開始
        ToCoNet_vMacStart();
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
    //今回は使わない
	if (!bAfterAhiInit) {
	} else {
		vInitHardware();
		ToCoNet_vMacStart();
	}
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
	switch(eEvent) {
	default:
		break;
	}
}


// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
    switch (u32DeviceId) {
    case E_AHI_DEVICE_TICK_TIMER:
    	break;

    default:
    	break;
    }
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	return FALSE;
}

