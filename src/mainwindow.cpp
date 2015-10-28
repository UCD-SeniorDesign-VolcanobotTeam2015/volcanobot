#include "../include/mainwindow.h"
#include "../build/ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->label->setStyleSheet("QLabel {background-color : white; }");
    ui->progressBar->setValue(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Browse_clicked()
{
    QStringList files = QFileDialog::getOpenFileNames(
                            this,
                            "Select one or more files to open",
                            "/home",
                            "Text files (*.oni)");
    if(files.size() > 0)
        ui->label->setText(files[0] + " selected");
    else {
        ui->label->setText("No .ONI file selected");
        ui->label->setAlignment(Qt::AlignTop);
    }
}

void MainWindow::on_Cancel_clicked()
{
   this->close();
}

void MainWindow::on_Start_clicked()
{

}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->label->setVisible(checked);
    ui->progressBar->setValue(ui->progressBar->value()+1);
}
