#include <Arduino.h>
#include <mcp2515.h>
#include <avr/wdt.h>
#include <SPI.h>

#define CAN0_INT 2
#define WBL_PIN 5
#define SERIAL_SPEED 115200
#define INTACTIVE_DELAY_SECONDS 3
#define MINIMUM_ENGINE_RUN_TIME_SECONDS 20
#define PRINT_STATUS_DURATION_S 30

MCP2515 mcp2515(10);

// volatile bool interrupt = false;
unsigned long lastPrint = millis();

struct EngineInfo
{
    unsigned long lastActivity;
    unsigned long engineStart;
    bool engineRunning;
    bool wblOn;
};

EngineInfo engInfo = {0, 0, false, false};

// void irqHandler()
//{
//     interrupt = true;
// }

void setup()
{

    pinMode(CAN0_INT, INPUT);
    pinMode(WBL_PIN, OUTPUT);
    digitalWrite(WBL_PIN, LOW);
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    // mcp2515.setConfigMode();
    mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF);
    mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);
    mcp2515.setFilter(MCP2515::RXF0, false, 0x280);
    mcp2515.setFilter(MCP2515::RXF1, false, 0x5C0);
    mcp2515.setListenOnlyMode();
    // Using built in interrupt handler sometimes freezes mcp2515 if ISF is called
    // mid SPI transfer it seems..
    // attachInterrupt(digitalPinToInterrupt(2), irqHandler, FALLING);
    Serial.begin(SERIAL_SPEED);
    Serial.println("WBL controller started");
    wdt_enable(WDTO_1S);
}

void loop()
{
    if (!digitalRead(CAN0_INT))
    {
        handleInterrupt();
    }

    if (millis() < engInfo.lastActivity)
    {
    }

    if ((millis() - engInfo.lastActivity) / 1000 >= INTACTIVE_DELAY_SECONDS && (engInfo.engineRunning || engInfo.wblOn))
    {
        engInfo.engineRunning = false;
        engInfo.engineStart = 0;
        if (engInfo.wblOn)
        {
            Serial.println("shutting off WBL due to inactivity");
            engInfo.wblOn = false;
            digitalWrite(WBL_PIN, LOW);
        }
    }

    if (engInfo.engineRunning && (millis() - engInfo.engineStart) / 1000 > MINIMUM_ENGINE_RUN_TIME_SECONDS)
    {
        if (!engInfo.wblOn)
        {
            Serial.println("wbl on");
            engInfo.wblOn = true;
            digitalWrite(WBL_PIN, HIGH);
        }
    }
    else
    {
        if (engInfo.wblOn)
        {
            Serial.println("wbl off");
            engInfo.wblOn = false;
            digitalWrite(WBL_PIN, LOW);
        }
    }
    if (millis() - lastPrint >= (PRINT_STATUS_DURATION_S * 1000) && engInfo.engineRunning)
    {
        printStats();
    }
    wdt_reset();
}

void printStats()
{

    lastPrint = millis();
    Serial.print("engine running for ");
    Serial.print((millis() - engInfo.engineStart) / 1000);
    Serial.println(" seconds");
}

void handleInterrupt()
{
    // interrupt = false;
    uint8_t irq = mcp2515.getInterrupts();
    mcp2515.clearInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF)
    {
        struct can_frame frame;
        if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK)
        {
            processFrame(&frame);
        }
    }
    if (irq & MCP2515::CANINTF_RX1IF)
    {
        struct can_frame frame;
        if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK)
        {
            processFrame(&frame);
        }
    }
    if ((irq & MCP2515::CANINTF_ERRIF) || (irq & MCP2515::CANINTF_MERRF))
    {
        checkErr();
    }
}

void processFrame(can_frame *frame)
{
    if (frame->can_id == 0x5C0)
    {
        int8_t temp = (int8_t)frame->data[1] - 40;
        Serial.print("coolant temp: ");
        Serial.print(temp);
        Serial.println(" C");
    }
    if (frame->can_id == 0x280)
    {
        engInfo.lastActivity = millis();
        if (frame->data[5] & 0x80)
        {
            if (!engInfo.engineRunning)
            {
                engInfo.engineStart = engInfo.lastActivity;
                engInfo.engineRunning = true;
            }
        }
        else
        {
            engInfo.engineRunning = false;
        }
    }
}

void serialPrint(can_frame *f)
{
    Serial.print(millis(), HEX);
    Serial.print(" ");
    Serial.print(f->can_id, HEX);
    Serial.print(":");
    for (int i = 0; i < f->can_dlc; i++)
    {
        char buf[3];
        sprintf(buf, "%02X", f->data[i]);
        Serial.print(buf);
    }
    Serial.println();
}

void checkErr()
{
    uint8_t u8ErrorFlag = mcp2515.getErrorFlags();
    if (u8ErrorFlag & MCP2515::EFLG_RX1OVR)
    {
        Serial.println("CanShield error RX1OVR: receive buffer 1 overflow");
    }

    // if (u8ErrorFlag & MCP2515::EFLG_RX0OVR)
    //{
    //     Serial.println("CanShield error RX0OVR: receive buffer 0 overflow");
    // }

    if (u8ErrorFlag & MCP2515::EFLG_TXBO)
    {
        Serial.println("CanShield error TXBO: bus off");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXEP)
    {
        Serial.println("CanShield error TXEP: transmit error-passive");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXEP)
    {
        Serial.println("CanShield error RXEP: receive error-passive");
    }

    if (u8ErrorFlag & MCP2515::EFLG_TXWAR)
    {
        Serial.println("CanShield error TXWAR: transmit error warning");
    }

    if (u8ErrorFlag & MCP2515::EFLG_RXWAR)
    {
        Serial.println("CanShield error RXWAR: receive error warning ");
    }

    if (u8ErrorFlag & MCP2515::EFLG_EWARN)
    {
        Serial.println("CanShield error EWARN: error warning ");
    }
}
