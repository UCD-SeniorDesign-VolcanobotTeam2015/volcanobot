#include "../include/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include "../include/oni-to-pcd.h"
#include <iostream>
#include "../include/CloudStitcher.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
//    ui->plainTextEdit->setStyleSheet("QLabel {background-color : white; }");
    ui->progressBar->setValue(0);
    counter = 0;
    ui->plainTextEdit->setReadOnly(true);
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
    if(files.size() > 0) {
	oniFileName = files[0];
        appendMessage(QString(oniFileName + " selected"));
	}
    else {
        appendMessage("No .ONI file selected");
//        ui->plainTextEdit->setAlignment(Qt::AlignTop);
    }
}

void MainWindow::on_Cancel_clicked()
{
   this->close();
}

void MainWindow::on_Start_clicked()
{
/*
 argv[2] contains path to output pcdfiles 
 dir contains where the pcdfiles will actually be output
 output contains where the final pointcloud file will be stored off of argv[2]
*/

    if(oniFileName == "") {
        appendMessage("ERROR: Please browse for an .ONI file before clicking start");
        return;
    }
int argc = 3; 
char* argv[3];
int length = strlen(oniFileName.toStdString().c_str());
std::string resFolder = "/home/paul/Documents/res";
argv[1] = new char[length + 1]();
strncpy(argv[1], oniFileName.toStdString().c_str(), length+1);

argv[2] = "/home/paul/Documents/res/pcdFiles";
std::cout <<argv[1] << "-"; // '-' shows ending characters
std::cout << "\n" << argv[2] << "-";
vba::oni2pcd::driver(argc, argv);

vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
std::string dir(argv[2]);
dir = dir + "/pcdTemp";
std::string output(resFolder + "/finalPointCloud");
mCloudStitcher->setOutputPath( output );
mCloudStitcher->stitchPCDFiles( dir );
delete mCloudStitcher;
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->progressBar->setValue(ui->progressBar->value()+1);
    toDisplay = "index: " + QString::number(++counter) + "\n" + toDisplay;
    // ui->label->setText(toDisplay);
    QString msg = "This is the message" + QString::number(++counter);
    appendMessage(msg);
    ui->plainTextEdit->setVisible(checked);

}


// ** Helper Functions ** //
void MainWindow::appendMessage(QString msg) {
    ui->plainTextEdit->appendPlainText(msg);

}

