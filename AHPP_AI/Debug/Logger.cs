using System;
using System.IO;

namespace AHPP_AI.Debug
{
    /// <summary>
    ///     Handles logging to both console and file with thread safety
    /// </summary>
    public class Logger
    {
        /// <summary>
        ///     Describes the severity of log messages for filtering output.
        /// </summary>
        public enum LogLevel
        {
            Trace = 1,
            Debug = 2,
            Info = 3,
            Warn = 4,
            Error = 5
        }

        private readonly bool consoleLogging = true;
        private readonly string filePath;
        private readonly object lockObject = new object();
        private LogLevel minimumLevel = LogLevel.Info;

        /// <summary>
        ///     Initializes a new logger that writes to the specified file
        /// </summary>
        /// <param name="filePath">Path to the log file</param>
        /// <param name="enableConsole">Whether to output to console as well</param>
        /// <param name="minimumLevel">Minimum log level to emit</param>
        public Logger(string filePath, bool enableConsole = true, LogLevel minimumLevel = LogLevel.Info)
        {
            this.filePath = ResolveLogFilePath(filePath);
            consoleLogging = enableConsole;
            this.minimumLevel = minimumLevel;

            try
            {
                var directory = Path.GetDirectoryName(this.filePath);
                if (!string.IsNullOrWhiteSpace(directory))
                    Directory.CreateDirectory(directory);

                File.AppendAllText(this.filePath,
                    $"{Environment.NewLine}=== Log session started at {DateTime.Now:yyyy-MM-dd HH:mm:ss.fff} ==={Environment.NewLine}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: Could not initialize log file: {ex.Message}");
            }
        }

        /// <summary>
        ///     Logs a message with timestamp to console and file
        /// </summary>
        /// <param name="message">The message to log</param>
        /// <param name="level">Optional log level (INFO, WARN, ERROR)</param>
        public void Log(string message, string level = "INFO")
        {
            if (string.IsNullOrEmpty(message)) return;

            var timestamp = DateTime.Now.ToString("HH:mm:ss.fff");
            var resolvedLevel = ParseLogLevel(level, LogLevel.Info);
            if (resolvedLevel < minimumLevel) return;

            var logMessage = $"{timestamp} [{FormatLogLevel(resolvedLevel)}] {message}";

            // Output to console if enabled
            if (consoleLogging)
            {
                Console.ForegroundColor = GetColorForLevel(resolvedLevel);
                Console.WriteLine(logMessage);
                Console.ResetColor();
            }

            // Write to file with lock to ensure thread safety
            try
            {
                lock (lockObject)
                {
                    File.AppendAllText(filePath, logMessage + Environment.NewLine);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: Could not write to log file: {ex.Message}");
            }
        }

        /// <summary>
        ///     Log a warning message
        /// </summary>
        public void LogWarning(string message)
        {
            Log(message, "WARN");
        }

        /// <summary>
        ///     Log an error message
        /// </summary>
        public void LogError(string message)
        {
            Log(message, "ERROR");
        }

        /// <summary>
        ///     Log an informational message.
        /// </summary>
        public void LogInfo(string message)
        {
            Log(message, "INFO");
        }

        /// <summary>
        ///     Log a debug message.
        /// </summary>
        public void LogDebug(string message)
        {
            Log(message, "DEBUG");
        }

        /// <summary>
        ///     Log a trace message.
        /// </summary>
        public void LogTrace(string message)
        {
            Log(message, "TRACE");
        }

        /// <summary>
        ///     Set the minimum log level that will be emitted.
        /// </summary>
        public void SetMinimumLevel(LogLevel level)
        {
            minimumLevel = level;
        }

        /// <summary>
        ///     Log an exception with stack trace
        /// </summary>
        public void LogException(Exception ex, string context = "")
        {
            var message = string.IsNullOrEmpty(context)
                ? $"Exception: {ex.Message}"
                : $"{context}: {ex.Message}";

            LogError(message);
            LogError($"Stack trace: {ex.StackTrace}");
        }

        /// <summary>
        ///     Parse a log level string to the closest matching log level.
        /// </summary>
        public static LogLevel ParseLogLevel(string level, LogLevel fallback)
        {
            if (string.IsNullOrWhiteSpace(level)) return fallback;

            switch (level.Trim().ToUpperInvariant())
            {
                case "TRACE":
                    return LogLevel.Trace;
                case "DEBUG":
                    return LogLevel.Debug;
                case "INFO":
                    return LogLevel.Info;
                case "WARN":
                case "WARNING":
                    return LogLevel.Warn;
                case "ERROR":
                    return LogLevel.Error;
                default:
                    return fallback;
            }
        }

        /// <summary>
        ///     Resolve the final log file path and default bare filenames into a daily file under Logs.
        /// </summary>
        private static string ResolveLogFilePath(string configuredPath)
        {
            var trimmedPath = string.IsNullOrWhiteSpace(configuredPath) ? "log.txt" : configuredPath.Trim();
            var configuredDirectory = Path.GetDirectoryName(trimmedPath);
            if (string.IsNullOrWhiteSpace(configuredDirectory))
            {
                var baseDirectory = AppDomain.CurrentDomain.BaseDirectory;
                var logsDirectory = Path.Combine(baseDirectory, "Logs");
                var fileNameWithoutExtension = Path.GetFileNameWithoutExtension(trimmedPath);
                var extension = Path.GetExtension(trimmedPath);

                if (string.IsNullOrWhiteSpace(extension))
                    extension = ".txt";

                return Path.Combine(logsDirectory,
                    $"{fileNameWithoutExtension}_{DateTime.Now:yyyy-MM-dd}{extension}");
            }

            return trimmedPath;
        }

        /// <summary>
        ///     Convert a log level to its uppercase label.
        /// </summary>
        private static string FormatLogLevel(LogLevel level)
        {
            return level.ToString().ToUpperInvariant();
        }

        /// <summary>
        ///     Get console color for different log levels
        /// </summary>
        private ConsoleColor GetColorForLevel(LogLevel level)
        {
            switch (level)
            {
                case LogLevel.Error:
                    return ConsoleColor.Red;
                case LogLevel.Warn:
                    return ConsoleColor.Yellow;
                case LogLevel.Debug:
                    return ConsoleColor.Cyan;
                case LogLevel.Trace:
                    return ConsoleColor.DarkGray;
                default:
                    return ConsoleColor.Gray;
            }
        }
    }
}
