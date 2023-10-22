#include "mainwindow.hpp"
#include "arch.hpp"

namespace view
{

bool IsDirectory(const QString path)
{
    return QDir(path).exists();
}

void Invoke(std::stop_token stop_token, MainWindow* base_ptr)
{
	base_ptr->m_core.GetStat(base_ptr->m_dir.rootPath(), base_ptr->m_table_data, stop_token);
}

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent),
	  m_view(this),
	  m_button("Select folder", this),
	  m_path(this),
	  m_table(1, 4, this),
	  m_status(this),
	  m_subdir(this),
	  m_total_stat(this)
{
	this->setFixedSize(1100, 600);

	m_view.setModel(&m_dir);
	m_view.setRootIndex(m_dir.setRootPath(arch::root_path));
	m_view.setGeometry(0, 50, 600, 400);

	m_button.setGeometry(10, 10, 100, 30);

	m_dialog.setFileMode(QFileDialog::Directory);
	m_dialog.setOption(QFileDialog::ShowDirsOnly);
	m_dialog.setDirectory(arch::root_path);

	m_path.setGeometry(120, 10, 800, 30);
	m_path.setText(QString("Path: %1").arg(arch::root_path));

	m_table.setGeometry(650, 50, 420, 400);

	m_status.setGeometry(10, 450, 180, 30);

	m_subdir.setGeometry(650, 450, 180, 30);
	m_subdir.setText("Subdirs: ");

	m_total_stat.setGeometry(650, 480, 350, 50);
	m_total_stat.setText("Total stat\r\nCount:   Size:   Avg: ");

	connect(&m_button, &QPushButton::released, this, &MainWindow::SpawnDialog);
	connect(&m_core, &fs::Core::FileStat, this, &MainWindow::UpdateTable);
	connect(&m_core, &fs::Core::DirStat, this, &MainWindow::UpdateDirCount);
	connect(&m_view, &QTreeView::clicked, this, &MainWindow::NewDirSelected);

	RequestFsStat();

	this->show();
}

void MainWindow::SpawnDialog()
{
	if (!m_dialog.exec()) return;

    const QString dir = m_dialog.selectedFiles().constFirst();

    m_view.setRootIndex(UpdatePath(dir));

	RequestFsStat();
}

void MainWindow::RequestFsStat()
{
	m_status.setText("Status: calculating statistics");
	m_table.clear();
	m_table_data.clear();
	m_table.setHorizontalHeaderLabels({"Group", "Count", "Total, bytes", "Avg, bytes"});

	m_core_thread = std::jthread(Invoke, this);
}

void MainWindow::UpdateTable()
{
	m_table.setRowCount(m_table_data.size());

	quint64 row = 0;
	quint64 total_size  = 0;
	quint64 total_count = 0;
	for (auto i = m_table_data.constBegin(); i != m_table_data.constEnd(); ++i)
	{
		m_table.setItem(row, 0, new QTableWidgetItem(i.key()));
		m_table.setItem(row, 1, new QTableWidgetItem(QString::number(i.value().count)));
		m_table.setItem(row, 2, new QTableWidgetItem(QString::number(i.value().size)));
		m_table.setItem(row, 3, new QTableWidgetItem(
			QString::number(i.value().size/(i.value().count > 0 ? i.value().count : 1))
		));

		++row;

		total_count += i.value().count;
		total_size += i.value().size;
	}

	m_total_stat.setText(
		QString("Total stat\r\nCount: %1 Size: %2, bytes Avg: %3, bytes")
		.arg(QString::number(total_count),
			 QString::number(total_size),
             QString::number(total_size/(total_count > 0 ? total_count : 1)))
	);
	m_status.setText("Status: done");
}

void MainWindow::UpdateDirCount(quint64 count)
{
	m_subdir.setText(QString("Subdirs: %1").arg(count));
}

void MainWindow::NewDirSelected(const QModelIndex &index)
{
    const QString path = m_dir.filePath(index);

    if (index.column() == 0 && IsDirectory(path))
	{
        UpdatePath(path);
		RequestFsStat();
	}
}

QModelIndex MainWindow::UpdatePath(const QString path)
{
    m_path.setText(QString("Path: %1").arg(path));

    return m_dir.setRootPath(path);
}

}
