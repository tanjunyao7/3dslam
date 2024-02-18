#include <QThread>
#include <QProcess>
#include <iostream>

class ROSLauncherThread : public QThread
{
public:
    void run() override
    {
        QProcess process;
        // process.start("roslaunch", QStringList() << " LidarSLAM "
        //   << " hokuyo_hector_slam_hector.launch");

        QStringList arguments;
        arguments << "-x"
                  << "bash"
                  << "-c"
                  << "roslaunch LidarSLAM hokuyo_full.launch";
                //   << "roslaunch LidarSLAM sim_full.launch";
        process.start("gnome-terminal", arguments);

        if (!process.waitForStarted())
        {
            // Error handling if the process failed to start
            std::cout << "STARTED" << std::endl;
            // ...
        }

        if (!process.waitForFinished())
        {
            // Error handling if the process failed to finish
            // ...
            std::cout << "IN PROGRESS" << std::endl;
        }
    }

    void terminate()
    { // Close the gnome-terminal window
        QProcess closeProcess;
        closeProcess.start("pkill", QStringList() << "-f"
                                                  << "gnome-terminal");
        closeProcess.waitForFinished();
    }
};