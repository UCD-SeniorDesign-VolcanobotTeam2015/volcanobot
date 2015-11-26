#include "../include/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include "../include/oni-to-pcd.h"
#include <iostream>
#include "../include/CloudStitcher.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->plainTextEdit->setStyleSheet("QLabel {background-color : white; }");
    ui->progressBar->setValue(0);
    counter = 0;
    ui->plainTextEdit->setReadOnly(true);
    ui->plainTextEdit->setCenterOnScroll(true);
    ui->plainTextEdit->ensureCursorVisible();
    this->outputBuffer = new boost::lockfree::spsc_queue<std::string>(200);
    this->done = false;

    outputMessageThread = new boost::thread(&MainWindow::processOutputQueue, this);

    // process the order in which we want tasks to be run
    connect(this, SIGNAL(appendToConsole(QString)), ui->plainTextEdit, SLOT(insertPlainText(QString)));
    connect(this, SIGNAL(start(int)), this, SLOT(nextStep(int)));
    connect(this, SIGNAL(oniToPCDFinished(int)), this, SLOT(nextStep(int)));
    connect(this, SIGNAL(appendToConsole(QString)), this, SLOT(ensureCursorVisible(QString)));
    outputFolderName = "";
    oniFileName = "";
    taskThread = NULL;
}

MainWindow::~MainWindow()
{
    delete outputBuffer;
    delete outputMessageThread;
    delete taskThread;
    delete ui;
    // TODO add cleanup for threads
}

void MainWindow::ensureCursorVisible(QString s){
    ui->plainTextEdit->ensureCursorVisible();
    return;
}

void MainWindow::nextStep(const int& step) {

    std::cout << "inside nextstep with " << step << " input\n";
    switch(step) {

        case oniToPCD:
            clearTaskThread();
            taskThread = new boost::thread(&MainWindow::oniToPCDController, this);
            break;
        case cloudStitcher :
            clearTaskThread();
            taskThread = new boost::thread(&MainWindow::cloudStitcherController, this);
            break;

    default :
        appendToConsole(QString("Error. Could not find nextStep instruction"));
        break;
    }

}

void MainWindow::clearTaskThread() {
    if(taskThread == NULL) {
        std::cout << "inside clear task with == NULL\n";
        return;
    }
    taskThread->join(); // this should return immeditly as the thread should already have finished if this function is called.
    std::cout << "inside taskThread != NULL \n";
    delete taskThread;
    taskThread = NULL;
}


void MainWindow::cloudStitcherController() {


    /*
     *  if(oniFileName == "") {
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
     */
std::cout << "inside cloudstitchercontroller " << boost::this_thread::get_id() << std::endl;
    vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;

    std::string pcdFilesToStitchDir(this->outputFolderName.toStdString() + "/pcdTemp");
    std::string stitchedOniOutputDir (this->outputFolderName.toStdString() + "/finalPointCloud");
    std::cout << "inside cloudstitcher and pcdFilestoSttichdir = " << pcdFilesToStitchDir << "-\n";
    std::cout << "inside cloudstitcher and stitchonioutputdir = " << stitchedOniOutputDir << "-\n";

    mCloudStitcher->setOutputPath( stitchedOniOutputDir );

    mCloudStitcher->setOutputBuffer(this->outputBuffer);

    // worker test
    //boost::thread workerThread(&MainWindow::processOutputQueue, this);
    //return;
    //workerThread.join();
    //return;
    //this->done = true;
    //checkOutputBuffer();
    //return;

    //make a function pointer out of your custom function that follows the signature that I declared in my component.
    //The function you create just has to follow the lines void myFunctionName( std::string output , bool is_error )
    //vba::outputFunction function_pointer = &appendMessage;

    //this is my setter that takes the function pointer and uses it for all output. Otherwise it will just print to std::cout
    //and std::cerr by default
    //mCloudStitcher->setOutputFunction( function_pointer );

    mCloudStitcher->stitchPCDFiles( pcdFilesToStitchDir );

    delete mCloudStitcher;
    std::cout << "done wtih mcloudstitchercontroller \n";
    return;
//    emit cloudStitcherFinished();
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
        appendMessage(oniFileName.toStdString() + " selected" );
	}
    else {
        appendMessage("No .ONI file selected");
    }
}

void MainWindow::on_Cancel_clicked()
{
   this->close();
}

void MainWindow::processOutputQueue(){
    std::string temp = "";
    boost::posix_time::seconds waitTime(5);
    while(!done || !this->outputBuffer->empty()){
        if(this->outputBuffer->empty()){
            boost::this_thread::sleep(waitTime);
        }
        else {
            if(this->outputBuffer->pop(temp)) {
                //this->appendMessage(temp, false);
                QString output = QString::fromStdString(temp);
                emit appendToConsole(output);
                temp = ""; // clear value for safety
            }
        }
    }
    return;
}


void MainWindow::oniToPCDController(){
    /*
     argv[2] contains path to output pcdfiles
     dir contains where the pcdfiles will actually be output
     output contains where the final pointcloud file will be stored off of argv[2]
    */
if(outputFolderName == ""){
    this->outputBuffer->push("No output directory selected. Please select an output folder where you would like the final oni to go.");
    return;
}

if(oniFileName == "") {
        this->outputBuffer->push("ERROR: Please browse for an .ONI file before clicking start");
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
vba::oni2pcd::setOutputBuffer(this->outputBuffer);
vba::oni2pcd::driver(argc, argv);
emit oniToPCDFinished(cloudStitcher);

}

void MainWindow::on_Start_clicked()
{
    std::cout << "inside start " << boost::this_thread::get_id() << std::endl;
    emit start(oniToPCD);

/*
 argv[2] contains path to output pcdfiles 
 dir contains where the pcdfiles will actually be output
 output contains where the final pointcloud file will be stored off of argv[2]
*/

//    if(oniFileName == "") {
//        appendMessage("ERROR: Please browse for an .ONI file before clicking start");
//        return;
//    }
//int argc = 3;
//char* argv[3];
//int length = strlen(oniFileName.toStdString().c_str());
//std::string resFolder = "/home/paul/Documents/res";
//argv[1] = new char[length + 1]();
//strncpy(argv[1], oniFileName.toStdString().c_str(), length+1);

//argv[2] = "/home/paul/Documents/res/pcdFiles";
//std::cout <<argv[1] << "-"; // '-' shows ending characters
//std::cout << "\n" << argv[2] << "-";


//vba::oni2pcd::driver(argc, argv);

//vba::CloudStitcher* mCloudStitcher = new vba::CloudStitcher;
//std::string dir(argv[2]);
//dir = dir + "/pcdTemp";
//std::string output(resFolder + "/finalPointCloud");
//mCloudStitcher->setOutputPath( output );

//mCloudStitcher->setOutputBuffer(this->outputBuffer);


// worker test
//boost::thread workerThread(&MainWindow::processOutputQueue, this);
//return;
//workerThread.join();
//return;
//this->done = true;
//checkOutputBuffer();
//return;

//make a function pointer out of your custom function that follows the signature that I declared in my component.
//The function you create just has to follow the lines void myFunctionName( std::string output , bool is_error )
//vba::outputFunction function_pointer = &appendMessage;

//this is my setter that takes the function pointer and uses it for all output. Otherwise it will just print to std::cout
//and std::cerr by default
//mCloudStitcher->setOutputFunction( function_pointer );


//mCloudStitcher->stitchPCDFiles( dir );
//delete mCloudStitcher;
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->progressBar->setValue(ui->progressBar->value()+1);
    ui->plainTextEdit->setVisible(checked);
}


// ** Helper Functions ** //
void MainWindow::appendMessage(std::string msg,const bool is_error) {
    QString output = QString::fromStdString(msg);
    ui->plainTextEdit->appendPlainText(output);
}
void MainWindow::on_Browse_output_clicked()
{
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                "/home",
                                                QFileDialog::ShowDirsOnly
                                                | QFileDialog::DontResolveSymlinks);

    if(dir.size() > 0) {
    outputFolderName = dir;
        appendMessage(outputFolderName.toStdString() + " selected for output\n", false);
    }
    else {
        appendMessage("No outputFolder Selected file selected\n", false);
    }
}
