#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <string>
#include <QPlainTextEdit>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/policies.hpp>
#include <boost/thread.hpp>
#include <../include/MeshConstructor.h>

namespace Ui {
class MainWindow;
}
//class MainWindow;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void testPass();

private:
    void appendMessage(std::string msg, const bool is_error = false);
    void processOutputQueue();

private slots:
    void on_Browse_clicked();
    void on_Cancel_clicked();

    void on_Start_clicked();

    void on_radioButton_toggled(bool checked);

    void cloudStitcherController();

    void nextStep(const int&);


    void on_Browse_output_clicked();

    void ensureCursorVisible(QString);

signals:

    void appendToConsole(QString msg);

    void start(int);
    void oniToPCDFinished(int);
    void cloudStitcherFinished(int);


private:

    // Const
    static const int ONITOPCD = 0;
    static const int CLOUDSTITCHER = 1;
    static const int MESHCONSTRUCTOR = 2;

    Ui::MainWindow *ui;
    QString oniFileName;
    QString toDisplay;
    QString outputFolderName;
    boost::lockfree::spsc_queue<std::string>* outputBuffer;


    boost::thread* outputMessageThread;
    boost::thread* taskThread;

    bool done;

    void checkOutputBuffer();

    void oniToPCDController();
    void meshConstructorController();
    void clearTaskThread();


};

#endif // MAINWINDOW_H
