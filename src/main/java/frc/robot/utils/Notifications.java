package frc.robot.utils;

import frc.robot.utils.libraries.Elastic;
import frc.robot.utils.libraries.Elastic.Notification;
import frc.robot.utils.libraries.Elastic.Notification.NotificationLevel;

public class Notifications {
  // Fake enum
  public static final int INFO = 0;
  public static final int WARNING = 1;
  public static final int ERROR = 2;

  static void send(int type, String title, String description) {
    send(type, title, description, -1); // Yes it does work it makes it infinite
  }

  static void send(int type, String title, String description, int desplayTimeMillis) {
    NotificationLevel level = NotificationLevel.INFO;
    switch (type) {
      case 0:
        level = NotificationLevel.INFO;
        break;
      case 1:
        level = NotificationLevel.WARNING;
        break;
      case 2:
        level = NotificationLevel.ERROR;
        break;
    }

    Notification notification = new Notification(level, title, description, desplayTimeMillis);
    Elastic.sendNotification(notification);
  }
}
