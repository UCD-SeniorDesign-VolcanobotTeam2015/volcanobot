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
    if(files.size() > 0) {
	oniFileName = files[0];
        ui->label->setText(oniFileName + " selected");
	}
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
/*
char *argv[2];
int length = strlen(filePath);
argv[1] = new char(length +1);
strncpy(argv[1], filePath, length); 
*/

int argc = 3; 
char* argv[3];
int length = strlen(oniFileName.toStdString().c_str());
std::string resFolder = "/home/paul/Documents/res";
argv[1] = new char[length + 1]();
strncpy(argv[1], oniFileName.toStdString().c_str(), length+1);

argv[2] = "/home/paul/Documents/res/pcdFiles";
std::cout <<argv[1] << "-";
std::cout << "\n" << argv[2] << "-";
vba::oni2pcd::driver(argc, argv);

/* 
matt's code
vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;

	std::string dir( argv[1] );
	std::string output( argv[2] );
	mCloudStitcher->setOutputPath( output );
	mCloudStitcher->stitchPCDFiles( dir );
	delete mCloudStitcher;
*/
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
    ui->label->setVisible(checked);
    ui->progressBar->setValue(ui->progressBar->value()+1);
}
