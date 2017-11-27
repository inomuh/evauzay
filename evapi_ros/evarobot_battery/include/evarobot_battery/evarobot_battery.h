#include "ros/ros.h"

#include <iostream>
#include <string>
#include <stdio.h>
#include <iomanip>
#include <stdlib.h>

#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "im_msgs/Battery.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>

#include "ErrorCodes.h"

char SEM_NAME[]= "i2c";
