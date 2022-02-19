#ifndef _CONTROL_SYSTEM_H_
#define _CONTROL_SYSTEM_H_

#include "api.h"
#include <vector>
#include <string>

/**
 * A class for displaying information and selection
 * screens on the brain and controller(s)
 */
class ControlSystem
{
public:

    /**
     * A class for reading a button on one or more controllers
     */
    class DigitalControl
    {
    public:
        /**
         * The mode of the DigitalControl
         */
        enum DigitalControlMode
        {
            PRIMARY, // Only use the primary controller
            SECONDARY, // Only use the secondary controller
            BOTH // Use both controllers
        };

        DigitalControl(const pros::controller_digital_e_t digital, pros::Controller *controller);
        DigitalControl(const pros::controller_digital_e_t digital, pros::Controller *controller, pros::Controller *controller2, const DigitalControlMode mode);
        bool getValue() const;
        bool isNewPress();
        bool isNewRelease();

    private:
        const pros::controller_digital_e_t digital;
        pros::Controller *controller, *controller2;
        const DigitalControlMode mode;
        bool readPress = false, readRelease = false;
    };

    /**
     * A statistic to be displayed on the brain screen
     */
    class BrainStat
    {
        friend class ControlSystem;

    public:
        BrainStat(const std::string prefix, const std::string postfix = "");
        BrainStat(const std::string prefix, std::vector<std::string> postfixes, const std::string postfix = "");
        void update(const std::string value);
        void update(std::vector<std::string> values);
        void update(const float value);

        std::string getLine() const;

    protected:
        const std::string prefix, postfix;
        std::string line;
        std::vector<std::string> postfixes;
        lv_obj_t *label;
    };

    ControlSystem(pros::Controller *controller);
    ControlSystem(pros::Controller *controller, pros::Controller *controller2);

    void controllersPrint(const int line, const int column, const char *text);
    static void controllersPrint(const int line, const int column, const char *text, pros::Controller *controller1, pros::Controller *controller2);

    void controllersClear();
    static void controllersClear(pros::Controller *controller1, pros::Controller *controller2);

    bool controllersRead(const pros::controller_digital_e_t button);
    static bool controllersRead(const pros::controller_digital_e_t button, pros::Controller *controller1, pros::Controller *controller2);

    void controllersRumble(const char* rumblePattern = ".");
    static void controllersRumble(const char* rumblePattern, pros::Controller *controller1, pros::Controller *controller2);

    void initBrainStats(lv_obj_t *parentObj = lv_scr_act());

    void printBrainStats();
    void addBrainStat(BrainStat *stat);
    void printControllerCompStats();
    int selectionScreen(const char* title, std::vector<const char*> options, const pros::controller_digital_e_t upButton = pros::E_CONTROLLER_DIGITAL_UP,
    const pros::controller_digital_e_t downButton = pros::E_CONTROLLER_DIGITAL_DOWN, const pros::controller_digital_e_t confirmButton = pros::E_CONTROLLER_DIGITAL_A);

    DigitalControl* addDigitalControl(pros::controller_digital_e_t digital, DigitalControl::DigitalControlMode mode);

private:
    pros::Controller *controller, *controller2;
    std::vector<DigitalControl*> digitalControls;

    BrainStat *batteryStat, *controllerStat, *competitionStat;

    bool labelsInitialized = false;
    lv_obj_t *statsParent, *batteryStatus, *controllerStatus, *competitionStatus;

    int lowestLabelLine = 0;
};

#endif // _CONTROL_SYSTEM_H_