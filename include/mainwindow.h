#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QFileDialog>
#include <string>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_Browse_clicked();
    void on_Cancel_clicked();

    void on_Start_clicked();

    void on_radioButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;
    QString oniFileName;

    static void myOutputFunction( std::string output , bool is_error );
};

#endif // MAINWINDOW_H
