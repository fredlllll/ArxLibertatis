/*
 * Copyright 2011-2013 Arx Libertatis Team (see the AUTHORS file)
 *
 * This file is part of Arx Libertatis.
 *
 * Arx Libertatis is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Arx Libertatis is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Arx Libertatis.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "crashreporter/ErrorReport.h"

#include "platform/Platform.h"

#include <algorithm>
#include <signal.h>

#if ARX_PLATFORM == ARX_PLATFORM_WIN32
// Win32
#include <windows.h>
#endif

// Qt
#include <QApplication>
#include <QMessageBox>
#include <QDesktopWidget>
#include <QDir>
#include <QProcess>
#include <QTime>
#include <QFile>
#include <QFileInfo>
#include <QFileInfoList>
#include <QThread>
#include <QXmlStreamWriter>
#include <QByteArray>

// Boost
#include <boost/date_time/microsec_time_clock.hpp>
#include <boost/date_time/time_duration.hpp>
#include <boost/date_time/posix_time/ptime.hpp>

#include "Configure.h"

#include "core/Version.h"

#include "crashreporter/Win32Utilities.h"
#include "crashreporter/tbg/TBG.h"

#include "io/fs/Filesystem.h"
#include "io/fs/FileStream.h"

#include "platform/Architecture.h"
#include "platform/OS.h"
#include "platform/Process.h"

#include "util/String.h"

ErrorReport::ErrorReport(const QString& sharedMemoryName)
	: m_RunningTimeSec(0)
	, m_ProcessMemoryUsage(0)
	, m_SharedMemoryName(sharedMemoryName)
	, m_pCrashInfo()
	, m_Username("CrashBot")
	, m_Password("WbAtVjS9")
{
#if ARX_HAVE_PRCTL && defined(DEBUG)
	// Allow debuggers to be attached to this process, for development purpose...
	prctl(PR_SET_PTRACER, 1, 0, 0, 0);
#endif
}

bool ErrorReport::Initialize()
{	
	// Create a shared memory object.
	m_SharedMemory = boost::interprocess::shared_memory_object(boost::interprocess::open_only, m_SharedMemoryName.toStdString().c_str(), boost::interprocess::read_write);

	// Map the whole shared memory in this process
	m_MemoryMappedRegion = boost::interprocess::mapped_region(m_SharedMemory, boost::interprocess::read_write);

	// Our SharedCrashInfo will be stored in this shared memory.
	m_pCrashInfo = (CrashInfo*)m_MemoryMappedRegion.get_address();

	if(m_MemoryMappedRegion.get_size() != sizeof(CrashInfo))
	{
		m_DetailedError = "The size of the memory mapped region does not match the size of the CrashInfo structure.";
		return false;
	}

	bool bMiscCrashInfo = GetMiscCrashInfo();
	if(!bMiscCrashInfo)
		return false;
	
	m_pCrashInfo->reporterStarted.post();

	m_ReportFolder = fs::path(util::loadString(m_pCrashInfo->crashReportFolder))
	                 / fs::path(m_CrashDateTime.toString("yyyy.MM.dd hh.mm.ss").toUtf8());

	if(!fs::create_directories(m_ReportFolder))
	{
		m_DetailedError = QString("Unable to create directory (%1) to store the crash report files.").arg(m_ReportFolder.string().c_str());
		return false;
	}

	return true;
}

bool ErrorReport::GetCrashDump(const fs::path & fileName) {
	
	m_ProcessMemoryUsage = m_pCrashInfo->memoryUsage;
	m_RunningTimeSec = m_pCrashInfo->runningTime;
	
	if(!getCrashDescription()) {
		return false;
	}
	
	// Add attached files from the report
	size_t nbFilesAttached = std::min(size_t(m_pCrashInfo->nbFilesAttached),
	                                  size_t(CrashInfo::MaxNbFiles));
	for(size_t i = 0; i < nbFilesAttached; i++) {
		AddFile(util::loadString(m_pCrashInfo->attachedFiles[i]));
	}
	
#if ARX_PLATFORM == ARX_PLATFORM_WIN32
	
	fs::path miniDumpTmpFile = util::loadString(m_pCrashInfo->miniDumpTmpFile);
	
	bool bHaveDump = false;
	if(!miniDumpTmpFile.empty() && fs::exists(miniDumpTmpFile)) {
		fs::path fullPath = m_ReportFolder / fileName;
		if(fs::rename(miniDumpTmpFile, fullPath)) {
			AddFile(fullPath);
			bHaveDump = true;
		}
	}
	
	return bHaveDump;
	
#else //  ARX_PLATFORM != ARX_PLATFORM_WIN32
	
	ARX_UNUSED(fileName);
	
	// TODO: Write core dump to 
	// fs::path fullPath = m_ReportFolder / fileName;
	
	return true;
	
#endif
	
}

bool ErrorReport::getCrashDescription() {
	
	m_ReportDescription = util::loadString(m_pCrashInfo->description).c_str();
	
#if ARX_PLATFORM == ARX_PLATFORM_WIN32
	
	// Open parent process handle
	HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, m_pCrashInfo->processId);
	if(hProcess == NULL)
	{
		m_DetailedError = QString("Unable to obtain an handle to the crashed process (Error %1).").arg(QString::number(GetLastError()));
		return false;
	}
	
	if(m_pCrashInfo->exceptionCode != 0)
	{
		QString exceptionStr = GetExceptionString(m_pCrashInfo->exceptionCode).c_str();
		if(!exceptionStr.isEmpty())
		{
			m_ReportDescription += "\nException code:\n  ";
			m_ReportDescription += exceptionStr;
			m_ReportDescription += "\n";
		}
	}

	std::string callStack, callstackTop;
	u32 callstackCrc;

	bool bCallstack = GetCallStackInfo(hProcess, HANDLE(m_pCrashInfo->threadHandle),
	                                   &m_pCrashInfo->contextRecord, callStack,
	                                   callstackTop, callstackCrc);
	if(!bCallstack) 
	{
		m_DetailedError = "A failure occured when obtaining information regarding the callstack.";
		return false;
	}
	
	m_ReportUniqueID = QString("[%1]").arg(QString::number(callstackCrc, 16).toUpper());
	
	m_ReportDescription += "\nCallstack:\n";
	m_ReportDescription += callStack.c_str();
	m_ReportTitle = QString("%1 %2").arg(m_ReportUniqueID, callstackTop.c_str());

	QString registers(GetRegisters(&m_pCrashInfo->contextRecord).c_str());
	if(!registers.isEmpty())
	{
		m_ReportDescription += "\nRegisters:\n";
		m_ReportDescription += registers;
	}
	
	CloseHandle(hProcess);
	
#else // ARX_PLATFORM != ARX_PLATFORM_WIN32
	
	if(m_pCrashInfo->crashId != 0) {
		m_ReportUniqueID = QString("[%1]").arg(QString::number(m_pCrashInfo->crashId, 16).toUpper());
	}
	
	std::string title = util::loadString(m_pCrashInfo->title);
	m_ReportTitle = QString("%1 %2").arg(m_ReportUniqueID, title.c_str());
	
#endif // ARX_PLATFORM != ARX_PLATFORM_WIN32
	
	return true;
}

bool ErrorReport::GetMiscCrashInfo() {
	
	// Get crash time
	m_CrashDateTime = QDateTime::currentDateTime();
	
	m_ProcessArchitecture = platform::getArchitectureName(m_pCrashInfo->architecture);
	
	m_OSName = QString::fromUtf8(platform::getOSName().c_str());
	m_OSArchitecture = QString::fromUtf8(platform::getOSArchitecture().c_str());
	m_OSDistribution = QString::fromUtf8(platform::getOSDistribution().c_str());

	return true;
}

bool ErrorReport::WriteReport(const fs::path & fileName) {
	
	fs::path fullPath = m_ReportFolder / fileName;
	
	QFile file(fullPath.string().c_str());
	if(!file.open(QIODevice::WriteOnly)) {
		m_DetailedError = "Unable to open report manifest for writing.";
		return false;
	}
	
	QXmlStreamWriter xml;
	xml.setDevice(&file);
	xml.setAutoFormatting(true);
	xml.writeStartDocument();
	xml.writeStartElement("CrashReport");
		
		xml.writeComment("Information related to the crashed process");
		xml.writeStartElement("Process");
			xml.writeTextElement("Path", util::loadString(m_pCrashInfo->executablePath).c_str());
			xml.writeTextElement("Version", util::loadString(m_pCrashInfo->executableVersion).c_str());
			if(m_ProcessMemoryUsage != 0) {
				xml.writeTextElement("MemoryUsage", QString::number(m_ProcessMemoryUsage));
			}
			xml.writeTextElement("Architecture", m_ProcessArchitecture);
			if(m_RunningTimeSec > 0) {
				xml.writeTextElement("RunningTime", QString::number(m_RunningTimeSec));
			}
			xml.writeTextElement("CrashDateTime", m_CrashDateTime.toString("dd.MM.yyyy hh:mm:ss"));
		xml.writeEndElement();

		xml.writeComment("Information related to the OS");
		xml.writeStartElement("OS");
			if(!m_OSName.isEmpty()) {
				xml.writeTextElement("Name", m_OSName);
			}
			if(!m_OSArchitecture.isEmpty()) {
				xml.writeTextElement("Architecture", m_OSArchitecture);
			}
			if(!m_OSDistribution.isEmpty()) {
				xml.writeTextElement("Distribution", m_OSDistribution);
			}
		xml.writeEndElement();

		xml.writeComment("List of files generated by the crash reporter");
		xml.writeComment("Note that some of these files could have been manually excluded from the report");
		xml.writeStartElement("Files");
		for(FileList::const_iterator it = m_AttachedFiles.begin();
		    it != m_AttachedFiles.end(); ++it) {
			xml.writeTextElement("File", it->path.string().c_str());
		}
		xml.writeEndElement();

		xml.writeComment("Variables attached by the crashed process");
		xml.writeStartElement("Variables");
		size_t nbVariables = std::min(size_t(m_pCrashInfo->nbVariables),
		                              size_t(CrashInfo::MaxNbVariables));
		for(size_t i = 0; i < nbVariables; ++i) {
			xml.writeStartElement("Variable");
			xml.writeAttribute("Name", util::loadString(m_pCrashInfo->variables[i].name).c_str());
			xml.writeAttribute("Value", util::loadString(m_pCrashInfo->variables[i].value).c_str());
			xml.writeEndElement();
		}
		xml.writeEndElement();
		
	xml.writeEndElement();
	xml.writeEndDocument();
	
	file.close();

	AddFile(fullPath);
	
	return true;
}

bool ErrorReport::GenerateReport(ErrorReport::IProgressNotifier* pProgressNotifier)
{
	ErrorReport* report = this;
	BOOST_SCOPE_EXIT((report))
	{
		// Allow the crashed process to exit
		report->ReleaseApplicationLock();
	} BOOST_SCOPE_EXIT_END

	pProgressNotifier->taskStarted("Generating crash report", 4);
	
	// Initialize shared memory
	pProgressNotifier->taskStepStarted("Connecting to crashed application");
	bool bInit = Initialize();
	pProgressNotifier->taskStepEnded();
	if(!bInit)
	{
		pProgressNotifier->setError("Could not generate the crash dump.");
		pProgressNotifier->setDetailedError(m_DetailedError);
		return false;
	}
	
	if(m_pCrashInfo->architecture != ARX_ARCH) {
		pProgressNotifier->setError("Architecture mismatch between the crashed process and the crash reporter.");
		pProgressNotifier->setDetailedError(m_DetailedError);
		return false;
	}
	
	// Wait for crash to be processed
	pProgressNotifier->taskStepStarted("Processing crash information");
	while(platform::isProcessRunning(m_pCrashInfo->processorProcessId)) {
		boost::posix_time::ptime timeout
		 = boost::posix_time::microsec_clock::universal_time()
		 + boost::posix_time::milliseconds(100);
		if(m_pCrashInfo->processorDone.timed_wait(timeout)) {
			break;
		}
	}
	pProgressNotifier->taskStepEnded();
	
	// Generate minidump
	pProgressNotifier->taskStepStarted("Generating crash dump");
	bool bCrashDump = GetCrashDump("crash.dmp");
	pProgressNotifier->taskStepEnded();
	if(!bCrashDump)
	{
		pProgressNotifier->setError("Could not generate the crash dump.");
		pProgressNotifier->setDetailedError(m_DetailedError);
		return false;
	}

	// Generate manifest
	pProgressNotifier->taskStepStarted("Generating report manifest");
	bool bCrashXml = WriteReport("crash.xml");
	pProgressNotifier->taskStepEnded();
	if(!bCrashXml)
	{
		pProgressNotifier->setError("Could not generate the manifest.");
		pProgressNotifier->setDetailedError(m_DetailedError);
		return false;
	}

	return true;
}

bool ErrorReport::SendReport(ErrorReport::IProgressNotifier* pProgressNotifier)
{
	int nbFilesToSend = 0;
	for(FileList::const_iterator it = m_AttachedFiles.begin(); it != m_AttachedFiles.end(); ++it) 
	{
		if(it->attachToReport)
			nbFilesToSend++;
	}

	pProgressNotifier->taskStarted("Sending crash report", 3 + nbFilesToSend);
	
	std::string userAgent = "Arx Libertatis Crash Reporter (" + arx_version + ")";
	TBG::Server server("https://bugs.arx-libertatis.org", userAgent);
	
	// Login to TBG server
	pProgressNotifier->taskStepStarted("Connecting to the bug tracker");
	bool bLoggedIn = server.login(m_Username, m_Password);
	pProgressNotifier->taskStepEnded();
	if(!bLoggedIn)
	{
		pProgressNotifier->setError("Could not connect to the bug tracker");
		pProgressNotifier->setDetailedError(server.getErrorString());
		return false;
	}
	
	// Look for existing issue
	int issue_id = -1;
	pProgressNotifier->taskStepStarted("Searching for existing issue");
	if(!m_ReportUniqueID.isEmpty()) {
		m_IssueLink = server.findIssue(m_ReportUniqueID, issue_id);
	}
	pProgressNotifier->taskStepEnded();
	
	// Create new issue if no match was found
	if(issue_id == -1) {
		
		pProgressNotifier->taskStepStarted("Creating new issue");
		m_IssueLink = server.createCrashReport(m_ReportTitle, m_ReportDescription,
		                                       m_ReproSteps, tbg_version_id, issue_id);
		if(m_IssueLink.isNull()) {
			pProgressNotifier->taskStepEnded();
			pProgressNotifier->setError("Could not create a new issue on the bug tracker");
			pProgressNotifier->setDetailedError(server.getErrorString());
			return false;
		}

		// Set OS
#if   ARX_PLATFORM == ARX_PLATFORM_WIN32
		int os_id = TBG::Server::OS_Windows;
#elif ARX_PLATFORM == ARX_PLATFORM_LINUX
		int os_id = TBG::Server::OS_Linux;
#elif ARX_PLATFORM == ARX_PLATFORM_MACOSX
		int os_id = TBG::Server::OS_MacOSX;
#elif ARX_PLATFORM == ARX_PLATFORM_BSD
		#if defined(__FreeBSD__)
		int os_id = TBG::Server::OS_FreeBSD;
		#else
		int os_id = TBG::Server::OS_BSD;
		#endif
#else
		int os_id = TBG::Server::OS_Other;
#endif
		server.setOperatingSystem(issue_id, os_id);

		// Set Architecture
		int arch_id;
		if(m_ProcessArchitecture == ARX_ARCH_NAME_X86_64)
			arch_id = TBG::Server::Arch_Amd64;
		else if(m_ProcessArchitecture == ARX_ARCH_NAME_X86)
			arch_id = TBG::Server::Arch_x86;
		else
			arch_id = TBG::Server::Arch_Other;
		server.setArchitecture(issue_id, arch_id);

		pProgressNotifier->taskStepEnded();
	}
	else
	{
		if(!m_ReproSteps.isEmpty())
		{
			pProgressNotifier->taskStepStarted("Duplicate issue found, adding information");
			bool bCommentAdded = server.addComment(issue_id, m_ReproSteps);
			pProgressNotifier->taskStepEnded();
			if(!bCommentAdded)
			{
				pProgressNotifier->setError("Failure occured when trying to add information to an existing issue");
				pProgressNotifier->setDetailedError(server.getErrorString());
				return false;
			}
		}
	}

	// Send files
	QString commonPath;
	for(FileList::const_iterator it = m_AttachedFiles.begin(); it != m_AttachedFiles.end(); ++it) 
	{
		// Ignore files that were removed by the user.
		if(!it->attachToReport)
			continue;

		// One more check to verify that the file still exists.
		if(!fs::exists(it->path))
			continue;

		pProgressNotifier->taskStepStarted(QString("Sending file \"%1\"").arg(it->path.filename().c_str()));
		QString path = it->path.parent().string().c_str();
		QString file = it->path.string().c_str();
		QString name = it->path.filename().c_str();
		if(server.attachFile(issue_id, file, name, m_SharedMemoryName)) {
			commonPath.clear();
		} else {
			m_failedFiles.append(file);
			if(it == m_AttachedFiles.begin()) {
				commonPath = path;
			} else if(path != commonPath) {
				commonPath.clear();
			}
		}
		pProgressNotifier->taskStepEnded();
	}
	if(!commonPath.isEmpty() && m_failedFiles.count() > 1) {
		m_failedFiles.clear();
		m_failedFiles.append(commonPath);
	}
	
	return true;
}

void ErrorReport::ReleaseApplicationLock() {
	// Kill the original, busy-waiting process.
	platform::killProcess(m_pCrashInfo->processorProcessId);
	m_pCrashInfo->exitLock.post();
	platform::killProcess(m_pCrashInfo->processId);
}

void ErrorReport::AddFile(const fs::path& fileName)
{
	// Do not include files that can't be found, and empty files...
	if(fs::exists(fileName) && fs::file_size(fileName) != 0)
		m_AttachedFiles.push_back(File(fileName, true));
}

ErrorReport::FileList& ErrorReport::GetAttachedFiles()
{
	return m_AttachedFiles;
}

const QString& ErrorReport::GetErrorDescription() const
{
	return m_ReportDescription;
}

const QString& ErrorReport::GetIssueLink() const
{
	return m_IssueLink;
}

void ErrorReport::SetLoginInfo(const QString& username, const QString& password)
{
	m_Username = username;
	m_Password = password;
}

void ErrorReport::SetReproSteps(const QString& reproSteps)
{
	m_ReproSteps = reproSteps;
}
