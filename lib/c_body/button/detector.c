static hub_button_t hub_buttons_pressed(hub_button_t button_candidates)
{
  hub_button_t pressed;
  hub_button_is_pressed(&pressed);
  return pressed & button_candidates;
}

static int wait_for_hub_buttons(hub_button_t button_candidates)
{
    hub_button_t pressed_button;
    int button_command = 0;

    pressed_button = hub_buttons_pressed(button_candidates);

    if(pressed_button & HUB_BUTTON_LEFT)    button_command += 1;
    if(pressed_button & HUB_BUTTON_RIGHT)   button_command += 2;
    if(pressed_button & HUB_BUTTON_CENTER)  button_command += 4;
    if(pressed_button & HUB_BUTTON_BT)      button_command += 8;

    return button_command;
}