#ifndef CORE_HPP
#define CORE_HPP

#include <QHash>
#include <QDirIterator>

#include <thread>

namespace fs
{

struct Stat
{
	quint64 count{0};
	quint64 size{0};

	Stat() = default;
	Stat(quint64 count, quint64 size);

	~Stat() = default;
};

class Core : public QObject
{
	Q_OBJECT
public:
	Core()  = default;
	~Core() = default;

	Core(const Core&) = default;

	void GetStat(const QString path, QHash<QString, fs::Stat>&, const std::stop_token&);
signals:
	void FileStat();
	void DirStat(quint64);
};

}
#endif