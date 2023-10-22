#include "core.hpp"

#include <QDir>

namespace fs
{

Stat::Stat(quint64 count, quint64 size) : count(count), size(size) { }

void Core::GetStat(const QString path,
				   QHash<QString, fs::Stat>& result,
				   const std::stop_token& stop_token)
{
	QDir::Filters flags = QDir::AllEntries | QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Hidden;
	QDirIterator it(path, flags, QDirIterator::Subdirectories);

	quint64 dir_count = 0;

	while (it.hasNext())
	{
        it.next();
		if (it.fileInfo().isFile())
		{
			QString ext = it.fileInfo().suffix();
			++result[ext].count;
			result[ext].size += it.fileInfo().size();
		}
		else if (it.fileInfo().isDir())
		{
			++dir_count;
		}

		if (stop_token.stop_requested())
		{
			return;
		}
	}

	emit FileStat();

	emit DirStat(dir_count);
}

}
