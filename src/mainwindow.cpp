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

void MainWindow::on_BrowseOni_clicked()
{
    QStringList files = QFileDialog::getOpenFileNames(
                            this,
                            "Select one or more files to open",
                            QDir::homePath(),
                            "Text files (*.oni)");
    if(files.size() > 0) {
	oniFileName = files[0];
        ui->label->setText(oniFileName + " selected");
        ui->label->setAlignment(Qt::AlignTop);
	}
    else {
        ui->label->setText("No .Oni file selected");
        ui->label->setAlignment(Qt::AlignTop);
    }

    return;
}

void MainWindow::on_Close_clicked()
{
   this->close();
   return;
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
    ui->label->setAlignment(Qt::AlignTop);
    return;
}

if (oniFileName == ""){
    ui->label->setText(ui->label->text() + "\nNo .oni file selected. Please select a .oni file to process.");
    ui->label->setAlignment(Qt::AlignTop);
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

vba::outputFunction function_pointer = myOutputFunction;

//this is my setter that takes the function pointer and uses it for all output. Otherwise it will just print to std::cout
//and std::cerr by default

mCloudStitcher->setOutputFunction( function_pointer );


mCloudStitcher->stitchPCDFiles( dir );
ui->label->setText(ui->label->text() + "\n\nCompleted.\n\n");
ui->label->setAlignment(Qt::AlignTop);
std::cout << "\n\nCompleted.\n\n";
delete mCloudStitcher;
delete[] argv[1];
delete[] argv[2];
return;
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->label->setVisible(checked);
    ui->progressBar->setValue(ui->progressBar->value()+1);
    return;
}

void MainWindow::myOutputFunction( std::string output , bool is_error )
{
	if( is_error == true ) {
		std::cerr << output;
    }

	else {
		std::cout << output;
    }
    return;
}

void MainWindow::on_BrowseOutput_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, "Open Directory",
                                                QDir::homePath(),
                                                QFileDialog::ShowDirsOnly
                                                | QFileDialog::DontResolveSymlinks);

    if(dir.size() > 0) {
    outputFolderName = dir;
        ui->label->setText(outputFolderName + " selected for output");
        ui->label->setAlignment(Qt::AlignTop);
    }
    else {
        ui->label->setText("No output folder selected.");
        ui->label->setAlignment(Qt::AlignTop);
    }

    return;
}

// run moc ../include/mainwindow.h -o moc_mainwindow.cpp from the build directory of the project to generate
#include "../build/moc_mainwindow.cpp"
