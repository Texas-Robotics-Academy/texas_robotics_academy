#include <gtk/gtk.h>
#include <ros/ros.h>
#include <cstdio>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <iostream>
#include <iomanip>


GtkApplication *app;
GtkWidget *timerLabel, *startButton, *resetButton;
ros::Publisher buttonPub, resetPub;

double begin;
bool timerActive;
int timeTaken;

static void updateTime() {
  // updates the time and the label
  timeTaken = (int)(ros::Time::now().toSec() - begin);
  int minutes = timeTaken / 60;
  int seconds = timeTaken % 60;
  gtk_label_set_text(GTK_LABEL(timerLabel), 
    (static_cast<std::ostringstream*>( &(std::ostringstream() << std::setw(2) << std::setfill('0') << minutes << ":" <<
     std::setw(2) << std::setfill('0') << seconds) ) -> str()).c_str());  
}

static void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  if (timerActive) { // can only send a finishing time if the official race timer has been started
    double posX = msg ->pose[1].position.x;
    double posY = msg ->pose[1].position.y;
    // make sure its within the finishing "box" and student took a reasonable amount of time to finish
    if (posX >= -0.15 && posX <= -0.05 && posY >= -0.1 && posY <= 0.1 && timeTaken >= 15)
      timerActive = false;
    }
  }

static void timerCallback(const std_msgs::Empty::ConstPtr &msg) {
  // mark beginning time and set timer to update
  begin = ros::Time::now().toSec(); 
  timerActive = true;
}

static void pressed(uint8_t num) {
  std_msgs::UInt8 msg;
  msg.data = num;
  buttonPub.publish(msg);
}

static void startButtonCallback() { pressed(1); }

static void resetButtonCallback() {
  // reset the race
  pressed(2);
  gtk_label_set_text(GTK_LABEL(timerLabel), "00:00");  
  timerActive = false;
  std_msgs::Empty message;
  resetPub.publish(message);
}



static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window, *buttonBox, *mainBox;
  GtkCssProvider *styles;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Timer");
  gtk_window_set_resizable (GTK_WINDOW (window), false);

  styles = gtk_css_provider_new();

  mainBox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
  gtk_container_add(GTK_CONTAINER(window), mainBox);

  timerLabel = gtk_label_new("00:00");
  gtk_style_context_add_class(gtk_widget_get_style_context(GTK_WIDGET(timerLabel)), "lcd");
  gtk_widget_set_halign(timerLabel, GTK_ALIGN_START);
  gtk_container_add (GTK_CONTAINER (mainBox), timerLabel);

  buttonBox = gtk_button_box_new(GTK_ORIENTATION_VERTICAL);
  gtk_container_add (GTK_CONTAINER(mainBox), buttonBox);

  startButton = gtk_button_new_with_label("Start");
  g_signal_connect(startButton, "clicked", G_CALLBACK(startButtonCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), startButton);

  resetButton = gtk_button_new_with_label("Reset");
  g_signal_connect(resetButton, "clicked", G_CALLBACK(resetButtonCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), resetButton);

  gtk_css_provider_load_from_data(styles,
    ".lcd { color: lime; background-color: black; font-family: monospace; font-size: 400%;}",
    -1, NULL);
  gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
    GTK_STYLE_PROVIDER(styles),
    GTK_STYLE_PROVIDER_PRIORITY_USER);

  gtk_widget_show_all (window);
}

int spinTimer(void *) {
  if (ros::ok()) {
    if (timerActive)
        updateTime();  
    ros::spinOnce();
    return true;
  } else {
    g_application_quit(G_APPLICATION(app));
    return false;
  }
}

int main (int argc, char **argv){
  ros::init(argc, argv, "timer");

  ros::NodeHandle node;
  ros::Subscriber timerSub = node.subscribe("/stimer", 1000, &timerCallback);
  ros::Subscriber positionSub = node.subscribe("/gazebo/model_states", 10, &modelStateCallback);

  buttonPub = node.advertise<std_msgs::UInt8>("/button", 1000);
  resetPub = node.advertise<std_msgs::Empty>("/reset", 1000);

  app = gtk_application_new ("edu.utexas.cs.robocamp2020.timer", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);

  g_timeout_add(10, spinTimer, NULL);

  int result = g_application_run(G_APPLICATION(app), argc, argv);

  g_object_unref (app);
  return result;
}