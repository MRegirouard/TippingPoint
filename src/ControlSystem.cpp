#include "ControlSystem.h"

/**
 * Construct a new DigitalControl with the given button and one controller
 * @param digital The button to read
 * @param controller The controller to read from
 */
ControlSystem::DigitalControl::DigitalControl(const pros::controller_digital_e_t digital, pros::Controller *controller)
: digital(digital), mode(PRIMARY)
{
	this->controller = controller;
}

/**
 * Construct a new DigitalControl with the given button and two controllers
 * @param digital The button to read
 * @param controller The first controller to read from
 * @param controller2 The second controller to read from
 * @param mode The mode of the DigitalControl
 */
ControlSystem::DigitalControl::DigitalControl(const pros::controller_digital_e_t digital, pros::Controller *controller, pros::Controller *controller2, const DigitalControlMode mode)
: digital(digital), mode(mode)
{
	this->controller = controller;
	this->controller2 = controller2;
}

/**
 * Gets the value of the DigitalControl, true for pressed, false for released
 * @return True if the button is pressed on one or more controllers (depending on the mode), false otherwise
 */
bool ControlSystem::DigitalControl::getValue() const
{
	switch (mode)
	{
	case PRIMARY:
		return controller->get_digital(digital);
	case SECONDARY:
		return controller2->get_digital(digital);
	case BOTH:
	default:
		return controller->get_digital(digital) || controller2->get_digital(digital);
	}
}

/**
 * Gets whether the DigitalControl has been pressed since the last time this function was called
 * @return True if the button is now pressed on one or more controllers (depending on the mode), false otherwise
 */
bool ControlSystem::DigitalControl::isNewPress()
{
	if (getValue())
	{
		if (!readPress)
		{
			readPress = true;
			return true;
		}

		return false;
	}
	else
	{
		readPress = false;
		return false;
	}
}

/**
 * Gets whether the DigitalControl has been released since the last time this function was called
 * @return True if the button is now released on one or more controllers (depending on the mode), false otherwise
 */
bool ControlSystem::DigitalControl::isNewRelease()
{
	if (!getValue())
	{
		if (!readRelease)
		{
			readRelease = true;
			return true;
		}

		return false;
	}
	else
	{
		readRelease = false;
		return false;
	}
}

/**
 * Construct a new BrainStat with the given prefix and postfix
 * @param prefix The prefix of the BrainStat, to put before the value
 * @param postfix The postfix of the BrainStat, to put after the value
 */
ControlSystem::BrainStat::BrainStat(const std::string prefix, const std::string postfix)
: prefix(prefix), postfix(postfix)
{
	postfixes = std::vector<std::string>();
}

/**
 * Construct a new BrainStat with the given prefix, postfixes, and final postfix
 * @param prefix The prefix of the BrainStat, to put before the value
 * @param postfixes The postfixes of the BrainStat, to put after each given value
 * @param postfix The final postfix of the BrainStat, to put at the end of the line
 */
ControlSystem::BrainStat::BrainStat(const std::string prefix, std::vector<std::string> postfixes, const std::string postfix)
: prefix(prefix), postfix(postfix)
{
	this->postfixes = postfixes;
}

/**
 * Updates the BrainStat with the given value and sets the label text accordingly
 * @param value The value to update the BrainStat with
 */
void ControlSystem::BrainStat::update(const std::string value)
{
	line = prefix + value + postfix;

	if (label != nullptr)
		lv_label_set_text(label, line.c_str());
}

/**
 * Updates the BrainStat with the given values and sets the label text accordingly
 * @param values The values to update the BrainStat with, placed before their
 * respective postfixes
 */
void ControlSystem::BrainStat::update(std::vector<std::string> values)
{
	std::string tmp = "";

	for (int i = 0; i < values.size(); i++)
	{
		tmp += values[i];

		if (i < postfixes.size())
			tmp += postfixes[i];

	}

	update(tmp);
}

/**
 * Updates the BrainStat with the given value and sets the label text accordingly
 * @param value The value to update the BrainStat with
 */
void ControlSystem::BrainStat::update(const float value)
{
	update(std::to_string(value));
}

/**
 * @brief Construct a new ControlSystem with the given controller
 * @param controller The controller
 */
ControlSystem::ControlSystem(pros::Controller *controller)
{
	this->controller = controller;
	this->digitalControls = std::vector<DigitalControl*>();
}

/**
 * Construct a new ControlSystem with the given controllers
 * @param controller The first controller
 * @param controller2 The second controller
 */
ControlSystem::ControlSystem(pros::Controller *controller, pros::Controller *controller2)
{
	this->controller = controller;
	this->controller2 = controller2;
	this->digitalControls = std::vector<DigitalControl*>();
}

/**
 * Print the given C-string to the contained controllers at the given line and column
 * @param line The line to print at, [0-2]
 * @param column The column to print at, [0-14]
 * @param text The text to print
 */
void ControlSystem::controllersPrint(const int line, const int column, const char *text)
{
	controllersPrint(line, column, text, controller, controller2);
}

/**
 * Prints the given C-string to the given controllers at the given line and column.
 * Checks if each controller exists and is connected before printing
 * @param line The line to print at, [0-2]
 * @param column The column to print at, [0-14]
 * @param text The text to print
 * @param controller1 The first controller to print to
 * @param controller2 The second controller to print to
 */
void ControlSystem::controllersPrint(const int line, const int column, const char *text, pros::Controller *controller1, pros::Controller *controller2)
{
	if (controller1 != nullptr && controller1->is_connected())
	{
		controller1->print(line, column, text);
		pros::delay(50);
	}

	if (controller2 != nullptr && controller2->is_connected())
	{
		controller2->print(line, column, text);
		pros::delay(50);
	}
}

/**
 * Clears the contained controllers
 */
void ControlSystem::controllersClear()
{
	controllersClear(controller, controller2);
}

/**
 * Clears the given controllers. Checks if each controller exists and is connected before clearing
 * @param controller1 The first controller to clear
 * @param controller2 The second controller to clear
 */
void ControlSystem::controllersClear(pros::Controller *controller1, pros::Controller *controller2)
{
	bool cleared = false;

	if (controller1 != nullptr && controller1->is_connected())
	{
		controller1->clear();
		cleared = true;
		pros::delay(50);
	}

	if (controller2 != nullptr && controller1->is_connected())
	{
		controller2->clear();
		cleared = true;
		pros::delay(50);
	}
}

/**
 * Checks if the button is pressed on any contained controller
 * @param button The button to check
 * @return True if any controller is pressed on the given button, false otherwise
 */
bool ControlSystem::controllersRead(const pros::controller_digital_e_t button)
{
	return controllersRead(button, controller, controller2);
}

/**
 * Checks if the button is pressed on the given controllers
 * @param button The button to check
 * @param controller1 The first controller to check
 * @param controller2 The second controller to check
 * @return True if the button is pressed on either of the given controllers, false otherwise
 */
bool ControlSystem::controllersRead(const pros::controller_digital_e_t button, pros::Controller *controller1, pros::Controller *controller2)
{
    bool pressed = false;

    if (controller1 != nullptr)
        pressed |= controller1->get_digital(button);

    if (controller2 != nullptr)
        pressed |= controller2->get_digital(button);

    return pressed;
}

/**
 * Vibrates the contained controllers
 * @param rumblePattern The rumble pattern to vibrate with
 */
void ControlSystem::controllersRumble(const char* rumblePattern)
{
	controllersRumble(rumblePattern, controller, controller2);
}

/**
 * Vibrates the given controllers
 * @param rumblePattern The rumble pattern to vibrate with
 * @param controller1 The first controller to vibrate
 * @param controller2 The second controller to vibrate
 */
void ControlSystem::controllersRumble(const char* rumblePattern, pros::Controller *controller1, pros::Controller *controller2)
{
	bool rumbled = false;

	if (controller1 != nullptr)
	{
		controller1->rumble(rumblePattern);
		rumbled = true;
	}

	if (controller2 != nullptr)
	{
		controller2->rumble(rumblePattern);
		rumbled = true;
	}

	if (rumbled)
		pros::delay(50);
}

/**
 * Initializes the battery, controllers, and competition BrainStats
 * under the given parent object
 * @param parentObj The parent object to initialize the BrainStats under
 */
void ControlSystem::initBrainStats(lv_obj_t *parentObj)
{
	statsParent = parentObj;

	batteryStat = new BrainStat("Battery: ");
	controllerStat = new BrainStat("Controllers: ");
	competitionStat = new BrainStat("Competition: ");
	addBrainStat(batteryStat);
	addBrainStat(controllerStat);
	addBrainStat(competitionStat);
}

/**
 * Prints the battery, controller, and competition BrainStats
 */
void ControlSystem::printBrainStats()
{
	batteryStat->update(std::to_string(pros::battery::get_capacity()) + "%, " + std::to_string(pros::battery::get_temperature()) + " *C");

	std::string tmp = "";

	if (controller != nullptr)
	{
		tmp += std::to_string(controller->get_battery_capacity()) + "%";

		if (controller2 != nullptr)
			tmp += ", " + std::to_string(controller2->get_battery_capacity()) + "%";
	}
	else
	{
		if (controller2 != nullptr)
			tmp += std::to_string(controller2->get_battery_capacity()) + "%";
		else
			tmp += "None";
	}

	controllerStat->update(tmp);

	tmp = "";

	if (pros::competition::is_connected())
	{
		tmp += (pros::competition::is_autonomous() ? "Auton" : "Driver");
		tmp += " ";
		tmp += (pros::competition::is_disabled() ? "Disabled" : "Enabled");
	}
	else
		tmp += "None";

	competitionStat->update(tmp);
}

/**
 * Adds the given BrainStat to the BrainStats, creating a label
 * to display the BrainStat's text
 * @param stat The BrainStat to add
 */
void ControlSystem::addBrainStat(BrainStat *stat)
{
	stat->label = lv_label_create(statsParent, NULL);
	lv_obj_set_pos(stat->label, 0, lowestLabelLine);
	lowestLabelLine += 20;
}

/**
 * Prints the controller and main battery levels to the
 * first line on the controllers
 */
void ControlSystem::printControllerCompStats()
{
	std::string tmp = "";

	if (controller != nullptr && controller->is_connected())
		tmp = "P " + std::to_string(controller->get_battery_capacity()) + "  ";
	else
		tmp = "No P  ";

	if (controller2 != nullptr && controller2->is_connected())
		tmp += "S " + std::to_string(controller2->get_battery_capacity()) + "  ";
	else
		tmp += "No S  ";

	tmp += "B " + std::to_string((int) pros::battery::get_capacity());

	controllersPrint(0, 0, tmp.c_str());
}

/**
 * Displays a selection screen of the given options using the contained controllers.
 * The user can use the up and down buttons to select an option, and the A button to
 * confirm it.
 * @param title The title of the selection screen
 * @param options The options to display
 * @param upButton The button to use for the up button
 * @param downButton The button to use for the down button
 * @param confirmButton The button to use for the confirm button
 * @return The index of the option selected
 */
int ControlSystem::selectionScreen(const char* title, std::vector<const char*> options, const pros::controller_digital_e_t upButton,
const pros::controller_digital_e_t downButton, const pros::controller_digital_e_t confirmButton)
{
	controllersClear();

	bool selectionMade = false; // Whether or not a selection has been made

    int selectedOption = 0; // Current selected option
    int firstLineOption = -1; // What is displayed on the first line. -1 is the title line

    bool needsUpdate = true; // Whether or not the screen needs to be updated

	long lastUpdate = pros::millis();

	do
    {
        if (controllersRead(upButton)) // Move the cursor up
        {
            needsUpdate = true;

            if (selectedOption > 0)
            {
                if (selectedOption == firstLineOption) // If the cursor is on the first line, shift everything down
                    firstLineOption--;

                selectedOption--;
            }
            else if (selectedOption == 0) // If we're on the 0th option and move up, display the title
                firstLineOption = -1;

            while (controllersRead(upButton)) // Wait for button to be released
                pros::delay(20);
        }
        else if (controllersRead(downButton)) // Move the cursor down
        {
            needsUpdate = true;

            if (selectedOption < options.size() - 1)
            {
                if (selectedOption == firstLineOption + 2) // If the cursor is on the last line, shift everyting up
                    firstLineOption++;

                selectedOption++;
            }

            while (controllersRead(downButton)) // Wait for button to be released
                pros::delay(20);
        }

        selectionMade = controllersRead(confirmButton); // Check if the confirm button is pressed

        if (needsUpdate || pros::millis() - lastUpdate > 3000) // Only update the display if it is necessary, or it's been a while since the last update
        {
			lastUpdate = pros::millis();

            for (int i = 0; i < 3; i++) // 3 lines
            {
            	if (firstLineOption + i == -1) // A "-1" first line is the menu title
                {
                    controllersPrint(i, 0, title);
                }
                else if (firstLineOption + i < options.size())
                {
                    std::string printStr;

                    if (firstLineOption + i == selectedOption) // Highlight the selected option
                        printStr = "> ";
                    else
                        printStr = "  ";

					try // Options may not contain actual values, don't break if so
					{
					   printStr += options[firstLineOption + i];
					}
					catch (const std::exception& e)
					{
						std::cerr << "Selection Screen Error: " << e.what() << std::endl;
					}

                    printStr.append(15 - printStr.length(), ' '); // Pad the string with spaces to avoid having to spend an extra 50ms clearing the screen

                    controllersPrint(i, 0, printStr.c_str());
                }
            }

            needsUpdate = false;
        }

        pros::delay(20);
    } while (!selectionMade);

    while (controllersRead(confirmButton)) // Wait for confirm button to be released
        pros::delay(20);

    return selectedOption;
}

/**
 * Creates a DigitalControl and adds it to the ControlSystem
 * @param digital The button for the new DigitalControl
 * @param mode The mode of the DigitalControl
 * @return The new DigitalControl
 */
ControlSystem::DigitalControl *ControlSystem::addDigitalControl(pros::controller_digital_e_t digital, DigitalControl::DigitalControlMode mode)
{
	DigitalControl *control = new DigitalControl(digital, controller);

	if (controller2 == nullptr)
		control = new DigitalControl(digital, controller);
	else
		control = new DigitalControl(digital, controller, controller2, mode);

	digitalControls.push_back(control);

	return control;
}