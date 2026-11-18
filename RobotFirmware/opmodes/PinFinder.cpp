// =====================================================================
//  GLITCHED (Team 190) - digital pin finder
//  File: RobotFirmware/opmodes/PinFinder.cpp
//
//  OpMode "Pin Finder" (TELEOP). Shows the live level of all seven general-purpose
//  digital GPIOs of the Hub (DIGITAL_1 .. DIGITAL_7 in BoardPins.h), so you can find
//  out which physical pin is wired to which GPIO, and whether a pin still works.
//
//  HOW TO USE
//   1  Hub on, robot still, nothing driving. Every pin has its internal pull-up on, so
//      an unconnected pin reads 1.
//   2  Touch a jumper from a Hub GND pin to the physical pin you want to identify
//      (for example the middle pin of port D2). The matching line on the screen drops
//      to 0 and "last low" names that GPIO.
//   3  If you touch a pin and NO line changes, that pin is not one of these seven
//      GPIOs, or the pin is damaged.
//   4  If a line stays at 0 or 1 whatever you do, tell me which one.
//
//  Reads only: it never drives any pin. Pins 15 and 16 are also the line array's select
//  lines (LINE_SEL_1 / LINE_SEL_2): unplug the line array before testing so nothing is
//  fighting over them.
//
//  STATUS: written against the NRL SDK v1.5.0 headers. NOT compiled or run on the robot yet.
// =====================================================================
#include "NRL.h"

constexpr int NUM_PINS = 7;

static const int   PINS[NUM_PINS]  = { DIGITAL_1, DIGITAL_2, DIGITAL_3, DIGITAL_4, DIGITAL_5, DIGITAL_6, DIGITAL_7 };
static const char *NAMES[NUM_PINS] = {
    "GPIO1  (board D4)",
    "GPIO2  (board D5)",
    "GPIO15 (board D6)",
    "GPIO16 (board D7)",
    "GPIO35 (board D3)",
    "GPIO36 (board D1)",
    "GPIO37 (board D2)",
};

static int         lastLevel[NUM_PINS];
static const char *lastLow = "none yet";

class PinFinder : public NRLOpMode
{
public:
    void init() override
    {
        for (int i = 0; i < NUM_PINS; i++)
        {
            pinMode(PINS[i], INPUT_PULLUP);
            lastLevel[i] = digitalRead(PINS[i]);
        }
    }

    void start() override
    {
        lastLow = "none yet";
    }

    void loop() override
    {
        for (int i = 0; i < NUM_PINS; i++)
        {
            const int lvl = digitalRead(PINS[i]);
            if (lvl == 0 && lastLevel[i] == 1) lastLow = NAMES[i];   // the pin that just went low
            lastLevel[i] = lvl;
            telemetry.addData(NAMES[i], (float)lvl);
        }
        telemetry.addData("last low", lastLow);
    }

    void stop() override {}
};

REGISTER_OPMODE(PinFinder, "Pin Finder", TELEOP);