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
    outputFolderName = "";
    oniFileName = "";
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
 argv[2] contains path to output pcdfiles 
 dir contains where the pcdfiles will actually be output
 output contains where the final pointcloud file will be stored off of argv[2]
*/

if(outputFolderName == ""){
    ui->label->setText(ui->label->text() + "\nNo output directory selected. Please select an output folder where you would like the final oni to go.");
    return;
}

// setup for oni-many-pcd files
int argc = 3; 
char* argv[3];
int length = strlen(oniFileName.toStdString().c_str());
argv[1] = new char[length + 1]();
strncpy(argv[1], oniFileName.toStdString().c_str(), length+1);

length = strlen(outputFolderName.toStdString().c_str());
argv[2] = new char[length + 1]();
strncpy(argv[2], outputFolderName.toStdString().c_str(), length+1);

std::cout <<argv[1] << "-"; // '-' shows ending characters
std::cout << "\n" << argv[2] << "-";
vba::oni2pcd::driver(argc, argv);

vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
std::string dir(argv[2]);
dir = dir + "/pcdTemp";
std::string output(outputFolderName.toStdString() + "/finalPointCloud");
mCloudStitcher->setOutputPath( output );

//make a function pointer out of your custom function that follows the signature that I declared in my component.
//The function you create just has to follow the lines void myFunctionName( std::string output , bool is_error )
vba::outputFunction function_pointer = &myOutputFunction;

//this is my setter that takes the function pointer and uses it for all output. Otherwise it will just print to std::cout
//and std::cerr by default
mCloudStitcher->setOutputFunction( function_pointer );


mCloudStitcher->stitchPCDFiles( dir );
std::cout << "made if back here\n";
delete mCloudStitcher;
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->label->setVisible(checked);
    ui->progressBar->setValue(ui->progressBar->value()+1);
}

void MainWindow::myOutputFunction( std::string output , bool is_error )
{
	if( is_error == true )
		std::cerr << output;

	else
		std::cout << output;
}

void MainWindow::on_Browse_output_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                "/home",
                                                QFileDialog::ShowDirsOnly
                                                | QFileDialog::DontResolveSymlinks);

    if(dir.size() > 0) {
    outputFolderName = dir;
        ui->label->setText(outputFolderName + " selected for output");
    }
    else {
        ui->label->setText("No outputFolder Selected file selected");
        ui->label->setAlignment(Qt::AlignTop);
    }
}
