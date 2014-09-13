#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "simparameters.h"
#include "controller.h"

MainWindow::MainWindow(Controller &cont, int fps, QWidget *parent) :
    QMainWindow(parent),
    cont_(cont),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->GLWidget->setController(&cont);
    simRunning_ = false;
    connect(&renderTimer_, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer_.start(1000/fps);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    close();
}

void MainWindow::setParametersFromUI()
{
    SimParameters params;

    params.simRunning = simRunning_;

    if(ui->explicitEulerButton->isChecked())
        params.integrator = SimParameters::TI_EXPLICIT_EULER;
    else if(ui->implicitEulerButton->isChecked())
        params.integrator = SimParameters::TI_IMPLICIT_EULER;
    else if(ui->midpointButton->isChecked())
        params.integrator = SimParameters::TI_IMPLICIT_MIDPOINT;
    else if(ui->velocityVerletButton->isChecked())
        params.integrator = SimParameters::TI_VELOCITY_VERLET;

    params.timeStep = ui->timeStepEdit->text().toDouble();
    params.NewtonTolerance = ui->newtonTolEdit->text().toDouble();
    params.NewtonMaxIters = ui->newtonMaxItersEdit->text().toInt();

    params.activeForces = 0;
    if(ui->gravityCheckBox->isChecked())
        params.activeForces |= SimParameters::F_GRAVITY;
    if(ui->springsCheckBox->isChecked())
        params.activeForces |= SimParameters::F_SPRINGS;
    if(ui->floorCheckBox->isChecked())
        params.activeForces |= SimParameters::F_FLOOR;
    if(ui->dampingStiffnessCheckBox->isChecked())
        params.activeForces |= SimParameters::F_DAMPING;

    params.gravityG = ui->gravityGEdit->text().toDouble();
    params.springStiffness = ui->springStiffnessEdit->text().toDouble();
    params.maxSpringStrain = ui->maxStrainEdit->text().toDouble();
    params.dampingStiffness = ui->dampingStiffnessEdit->text().toDouble();

    if(ui->addParticleButton->isChecked())
        params.clickMode = SimParameters::CM_ADDPARTICLE;
    else if(ui->addSawButton->isChecked())
        params.clickMode = SimParameters::CM_ADDSAW;

    params.particleMass = ui->massEdit->text().toDouble();
    params.maxSpringDist = ui->maxSpringDistEdit->text().toDouble();
    params.particleFixed = ui->isFixedCheckBox->isChecked();

    params.sawRadius = ui->radiusEdit->text().toDouble();

    setUIFromParameters(params);
    QMetaObject::invokeMethod(&cont_, "updateParameters", Q_ARG(SimParameters, params));
}

void MainWindow::setUIFromParameters(const SimParameters &params)
{
    if(params.simRunning)
    {
        ui->startSimulationButton->setText(QString("Pause Simulation"));
        simRunning_ = true;
    }
    else
    {
        ui->startSimulationButton->setText(QString("Start Simulation"));
        simRunning_ = false;
    }

    switch(params.integrator)
    {
    case SimParameters::TI_EXPLICIT_EULER:
        ui->explicitEulerButton->setChecked(true);
        break;
    case SimParameters::TI_IMPLICIT_EULER:
        ui->implicitEulerButton->setChecked(true);
        break;
    case SimParameters::TI_IMPLICIT_MIDPOINT:
        ui->midpointButton->setChecked(true);
        break;
    case SimParameters::TI_VELOCITY_VERLET:
        ui->velocityVerletButton->setChecked(true);
        break;
    }

    ui->timeStepEdit->setText(QString::number(params.timeStep));
    ui->newtonTolEdit->setText(QString::number(params.NewtonTolerance));
    ui->newtonMaxItersEdit->setText(QString::number(params.NewtonMaxIters));

    ui->gravityCheckBox->setChecked(params.activeForces & SimParameters::F_GRAVITY);
    ui->springsCheckBox->setChecked(params.activeForces & SimParameters::F_SPRINGS);
    ui->floorCheckBox->setChecked(params.activeForces & SimParameters::F_FLOOR);
    ui->dampingStiffnessCheckBox->setChecked(params.activeForces & SimParameters::F_DAMPING);

    ui->gravityGEdit->setText(QString::number(params.gravityG));
    ui->springStiffnessEdit->setText(QString::number(params.springStiffness));
    ui->maxStrainEdit->setText(QString::number(params.maxSpringStrain));
    ui->dampingStiffnessEdit->setText(QString::number(params.dampingStiffness));

    if(params.clickMode == SimParameters::CM_ADDPARTICLE)
        ui->addParticleButton->setChecked(true);
    else if(params.clickMode == SimParameters::CM_ADDSAW)
        ui->addSawButton->setChecked(true);

    ui->massEdit->setText(QString::number(params.particleMass));
    ui->maxSpringDistEdit->setText(QString::number(params.maxSpringDist));
    ui->isFixedCheckBox->setChecked(params.particleFixed);
    ui->radiusEdit->setText(QString::number(params.sawRadius));
}

void MainWindow::updateGL()
{
    ui->GLWidget->update();
}

void MainWindow::on_actionReset_Everything_triggered()
{
    QMetaObject::invokeMethod(&cont_, "reset");
}

void MainWindow::on_actionReset_triggered()
{
    QMetaObject::invokeMethod(&cont_, "clearScene");
}

void MainWindow::on_startSimulationButton_clicked()
{
    simRunning_ = !simRunning_;
    setParametersFromUI();
}

void MainWindow::on_explicitEulerButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_implicitEulerButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_midpointButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_velocityVerletButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_timeStepEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonTolEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_newtonMaxItersEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_gravityCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_springsCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_floorCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_dampingStiffnessCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_gravityGEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_springStiffnessEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_maxStrainEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_dampingStiffnessEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_addParticleButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_addSawButton_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_massEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_maxSpringDistEdit_editingFinished()
{
    setParametersFromUI();
}

void MainWindow::on_isFixedCheckBox_clicked()
{
    setParametersFromUI();
}

void MainWindow::on_radiusEdit_editingFinished()
{
    setParametersFromUI();
}
