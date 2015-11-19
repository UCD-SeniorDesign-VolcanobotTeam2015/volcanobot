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
    void appendMessage(const std::string msg, const bool is_error = false);
    void processOutputQueue();

private slots:
    void on_Browse_clicked();
    void on_Cancel_clicked();

    void on_Start_clicked();

    void on_radioButton_toggled(bool checked);

signals:
    void appendToConcel(QString msg);

private:
    Ui::MainWindow *ui;
    QString oniFileName;
    QString toDisplay;
    int counter;
    boost::lockfree::spsc_queue<std::string>* outputBuffer;
//    boost::lockfree::spsc_queue<std::string> a(200);
//    boost::atomic<bool> done;
    bool done;

    void myOutputFunction( std::string output , bool is_error );
    void checkOutputBuffer();
};

#endif // MAINWINDOW_H
