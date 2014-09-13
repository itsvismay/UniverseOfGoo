#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

class Controller;
struct SimParameters;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(Controller &cont, int fps, QWidget *parent = 0);
    ~MainWindow();   

public slots:
    void setUIFromParameters(const SimParameters &params);

private slots:
    void updateGL();

    void on_actionExit_triggered();

    void on_actionReset_Everything_triggered();

    void on_actionReset_triggered();

    void on_startSimulationButton_clicked();

    void on_explicitEulerButton_clicked();

    void on_implicitEulerButton_clicked();

    void on_midpointButton_clicked();

    void on_velocityVerletButton_clicked();

    void on_timeStepEdit_editingFinished();

    void on_newtonTolEdit_editingFinished();

    void on_newtonMaxItersEdit_editingFinished();

    void on_gravityCheckBox_clicked();

    void on_springsCheckBox_clicked();

    void on_floorCheckBox_clicked();

    void on_dampingStiffnessCheckBox_clicked();

    void on_gravityGEdit_editingFinished();

    void on_springStiffnessEdit_editingFinished();

    void on_maxStrainEdit_editingFinished();

    void on_dampingStiffnessEdit_editingFinished();

    void on_addParticleButton_clicked();

    void on_addSawButton_clicked();

    void on_massEdit_editingFinished();

    void on_maxSpringDistEdit_editingFinished();

    void on_isFixedCheckBox_clicked();

    void on_radiusEdit_editingFinished();

private:
    Controller &cont_;
    Ui::MainWindow *ui;
    bool simRunning_;
    QTimer renderTimer_;

    void setParametersFromUI();
};

#endif // MAINWINDOW_H
