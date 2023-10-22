#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QFileSystemModel>
#include <QTreeView>
#include <QPushButton>
#include <QFileDialog>
#include <QLabel>
#include <QTableWidget>

#include <thread>

#include "core.hpp"

namespace view
{

using QTableType = QHash<QString, fs::Stat>;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = nullptr);

public slots:
	void UpdateTable();
	void UpdateDirCount(quint64);

private slots:
	void SpawnDialog();
	void NewDirSelected(const QModelIndex&);

private:
	void RequestFsStat();
    QModelIndex UpdatePath(const QString);

	QFileSystemModel m_dir;
	QTreeView        m_view;
	QPushButton      m_button;
	QFileDialog      m_dialog;
	QLabel           m_path;
	QTableWidget     m_table;
	QLabel           m_status;
	QLabel           m_subdir;
	QLabel           m_total_stat;
	QTableType       m_table_data;
	fs::Core         m_core;
	std::jthread     m_core_thread;

friend void Invoke(std::stop_token, MainWindow*);
};

}
#endif
