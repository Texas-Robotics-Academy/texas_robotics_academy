#include <gtk/gtk.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

GtkApplication *app;
GtkWidget *lcdLabel, *button1, *button2, *button3, *resetButton;
ros::Publisher buttonPub, resetPub;
std::string lines[2] = {"", ""};
#define NUM_LINES 2
#define LINE_LENGTH 32

static void setLine(unsigned int index, std::string message) {
  lines[index] = message;

  if (lcdLabel != NULL) {
    std::string text;
    for (int i = 0; i < NUM_LINES; i++) {
      if (i != 0) {
        text.append("\n");
      }
      std::string line = lines[i];
      line.resize(LINE_LENGTH, ' ');
      text.append(line);
    }
    gtk_label_set_text(GTK_LABEL(lcdLabel), text.c_str());
  }
}

static void lcd1Callback(std_msgs::String msg) {
  setLine(0, msg.data);
}
static void lcd2Callback(std_msgs::String msg) {
  setLine(1, msg.data);
}

static void pressed(uint8_t num) {
  std_msgs::UInt8 msg;
  msg.data = num;
  buttonPub.publish(msg);
}
static void buttonReleaseCallback() { pressed(0); }
static void button1Callback() { pressed(1); }
static void button2Callback() { pressed(2); }
static void button3Callback() { pressed(3); }

static void resetButtonCallback() {
  std_msgs::Empty msg;
  resetPub.publish(msg);
}

static void
activate (GtkApplication *app,
          gpointer        user_data)
{
  GtkWidget *window;
  GtkWidget *labelBox, *buttonBox, *mainBox;
  GtkCssProvider *styles;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "TexBot Virtual LCD");
  gtk_window_set_resizable (GTK_WINDOW (window), false);

  styles = gtk_css_provider_new();

  mainBox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
  gtk_container_add(GTK_CONTAINER(window), mainBox);

  lcdLabel = gtk_label_new("");
  setLine(0, "TexBot");
  setLine(1, "RoboCamp 2020");
  gtk_style_context_add_class(gtk_widget_get_style_context(GTK_WIDGET(lcdLabel)), "lcd");
//  gtk_widget_set_halign(lcdLabel, GTK_ALIGN_START);
  gtk_container_add (GTK_CONTAINER (mainBox), lcdLabel);

  buttonBox = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
  gtk_container_add (GTK_CONTAINER(mainBox), buttonBox);

  button1 = gtk_button_new_with_label("Button 1");
  g_signal_connect(button1, "pressed", G_CALLBACK(button1Callback), NULL);
  g_signal_connect(button1, "released", G_CALLBACK(buttonReleaseCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), button1);

  button2 = gtk_button_new_with_label("Button 2");
  g_signal_connect(button2, "pressed", G_CALLBACK(button2Callback), NULL);
  g_signal_connect(button2, "released", G_CALLBACK(buttonReleaseCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), button2);

  button3 = gtk_button_new_with_label("Button 3");
  g_signal_connect(button3, "pressed", G_CALLBACK(button3Callback), NULL);
  g_signal_connect(button3, "released", G_CALLBACK(buttonReleaseCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), button3);

  resetButton = gtk_button_new_with_label("Reset Robot");
  g_signal_connect(resetButton, "clicked", G_CALLBACK(resetButtonCallback), NULL);
  gtk_container_add(GTK_CONTAINER(buttonBox), resetButton);

  gtk_css_provider_load_from_data(styles,
    ".lcd { color: lime; background-color: black; font-family: monospace; font-size: 150%;}",
    -1, NULL);
  gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
    GTK_STYLE_PROVIDER(styles),
    GTK_STYLE_PROVIDER_PRIORITY_USER);

  gtk_widget_show_all (window);
}

int spinTimer(void *) {
  if (ros::ok()) {
    ros::spinOnce();
    return true;
  } else {
    g_application_quit(G_APPLICATION(app));
    return false;
  }
}

int main (int argc, char **argv){
  ros::init(argc, argv, "display");

  ros::NodeHandle node;
  ros::Subscriber line1Sub = node.subscribe("/lcd1", 5, &lcd1Callback);
  ros::Subscriber line2Sub = node.subscribe("/lcd2", 5, &lcd2Callback);

  buttonPub = node.advertise<std_msgs::UInt8>("/button", 5);
  resetPub = node.advertise<std_msgs::Empty>("/reset", 5);

  app = gtk_application_new ("edu.utexas.cs.robocamp2020", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);

  g_timeout_add(10, spinTimer, NULL);

  int result = g_application_run(G_APPLICATION(app), argc, argv);

  g_object_unref (app);
  return result;
}

