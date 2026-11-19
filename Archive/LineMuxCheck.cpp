// =====================================================================
//  GLITCHED (Team 190) - line array check (simple version)
//  File: RobotFirmware/opmodes/LineMuxCheck.cpp
//
//  OpMode "Line Mux Check" (TELEOP). Never moves the robot. It talks to the line array
//  directly (not through the line library) to find out whether the problem is the Hub's
//  wires/pins or the array itself.
//
//  The array has 8 sensors but only one signal wire. Three "select" wires tell it which
//  sensor to send. This OpMode asks for each of the 8 sensors in turn, then:
//    - checks that the three select wires really carry what the Hub asked for, and
//    - reads the sensor value (0 to 4095).
//
//  SCREEN (8 lines)
//    pins         OK, or BAD with the wire that is wrong (S0, S1 or S2)
//    ch 0 1       sensor 0 and sensor 1 readings
//    ch 2 3       sensor 2 and sensor 3
//    ch 4 5       sensor 4 and sensor 5
//    ch 6 7       sensor 6 and sensor 7
//    lowest       the smallest of the 8 readings
//    full (4095)  how many sensors read the maximum value
//    hint         FIX PINS / CHECK ARRAY / LOOKS OK (a rough guide, not a certainty)
//
//  HOW TO USE
//   1  Upload and start it. Hold the array a few millimetres over plain WHITE floor.
//      Write down the four ch lines and the "pins" line.
//   2  Slide a black card (or black tape on white) under the whole array. Write them down.
//   3  Cover the array with your hand. Write them down.
//
//  Select wires: S0 = GPIO18, S1 = GPIO15 (board port D6), S2 = GPIO16 (board port D7).
//  Signal wire: GPIO17. Keep anything else OFF ports D6 and D7 while this runs.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"

// ---------------------------------------------------------------------
//  PIN SET. true  = TEST WIRING: the four array wires moved to other board ports (see the
//                   table below), to find out whether the Hub's normal line pins are the fault.
//           false = NORMAL WIRING: the pins the line array library uses.
//
//  Test wiring (raw GPIO numbers; board port labels measured with Pin Finder):
//      array S0  wire -> board port D5  (GPIO2)
//      array S1  wire -> board port D3  (GPIO35)
//      array S2  wire -> board port D2  (GPIO37)     (unplug the encoder wire from D2 first)
//      array SIG wire -> board port D4  (GPIO1, an ADC1 pin: no WiFi conflict)
//  Power and ground stay where they are. Use the pin you touched with the jumper in Pin Finder
//  for each port. The "pins" line below tells you if the Hub really drives those pins.
// ---------------------------------------------------------------------
constexpr bool USE_TEST_PINS = true;

constexpr int SEL_S0    = USE_TEST_PINS ? 2  : 18;   // normal: BoardPins.h LINE_SEL_0
constexpr int SEL_S1    = USE_TEST_PINS ? 35 : 15;   // normal: LINE_SEL_1 (board D6)
constexpr int SEL_S2    = USE_TEST_PINS ? 37 : 16;   // normal: LINE_SEL_2 (board D7)
constexpr int SIG_PIN   = USE_TEST_PINS ? 1  : 17;   // normal: LINE_SIG, mux common -> ADC2_CH6
constexpr int SAMPLES   = 8;
constexpr int SETTLE_US = 200;   // longer than the library's 15 us, to rule out settling
constexpr int FULL_RAW  = 4090;  // at or above this counts as "full"

static char pinsLine[16];
static char pairLine[4][16];
static char lowLine[16];
static char fullLine[16];
static const char *hintLine = "-";

class LineMuxCheck : public NRLOpMode
{
public:
    void init() override
    {
        pinMode(SEL_S0, OUTPUT);
        pinMode(SEL_S1, OUTPUT);
        pinMode(SEL_S2, OUTPUT);
        pinMode(SIG_PIN, INPUT);
        snprintf(pinsLine, sizeof(pinsLine), "-");
        for (int i = 0; i < 4; i++) snprintf(pairLine[i], sizeof(pairLine[i]), "-");
        snprintf(lowLine, sizeof(lowLine), "-");
        snprintf(fullLine, sizeof(fullLine), "-");
    }

    void start() override {}

    void loop() override
    {
        int raw[8];
        int badMask = 0;     // bit 0 = S0 wrong, bit 1 = S1 wrong, bit 2 = S2 wrong

        for (int ch = 0; ch < 8; ch++)
        {
            digitalWrite(SEL_S0, (ch & 1) ? HIGH : LOW);
            digitalWrite(SEL_S1, (ch & 2) ? HIGH : LOW);
            digitalWrite(SEL_S2, (ch & 4) ? HIGH : LOW);
            delayMicroseconds(SETTLE_US);

            const int back = (digitalRead(SEL_S2) << 2) | (digitalRead(SEL_S1) << 1) | digitalRead(SEL_S0);
            badMask |= (back ^ ch);

            long sum = 0;
            for (int i = 0; i < SAMPLES; i++) sum += analogRead(SIG_PIN);
            raw[ch] = (int)(sum / SAMPLES);
        }

        // pins line
        if (badMask == 0) snprintf(pinsLine, sizeof(pinsLine), "OK");
        else snprintf(pinsLine, sizeof(pinsLine), "BAD%s%s%s",
                      (badMask & 1) ? " S0" : "", (badMask & 2) ? " S1" : "", (badMask & 4) ? " S2" : "");

        // readings
        int lowest = 4095, fullCount = 0;
        for (int i = 0; i < 8; i++)
        {
            if (raw[i] < lowest) lowest = raw[i];
            if (raw[i] >= FULL_RAW) fullCount++;
        }
        for (int p = 0; p < 4; p++)
            snprintf(pairLine[p], sizeof(pairLine[p]), "%d  %d", raw[2 * p], raw[2 * p + 1]);
        snprintf(lowLine, sizeof(lowLine), "%d", lowest);
        snprintf(fullLine, sizeof(fullLine), "%d of 8", fullCount);

        if (badMask != 0)        hintLine = "FIX PINS";
        else if (fullCount >= 4) hintLine = "CHECK ARRAY";
        else                     hintLine = "LOOKS OK";

        telemetry.addData("pins", pinsLine);
        telemetry.addData("ch 0 1", pairLine[0]);
        telemetry.addData("ch 2 3", pairLine[1]);
        telemetry.addData("ch 4 5", pairLine[2]);
        telemetry.addData("ch 6 7", pairLine[3]);
        telemetry.addData("lowest", lowLine);
        telemetry.addData("full (4095)", fullLine);
        telemetry.addData("hint", hintLine);
    }

    void stop() override {}
};

REGISTER_OPMODE(LineMuxCheck, "Line Mux Check", TELEOP);