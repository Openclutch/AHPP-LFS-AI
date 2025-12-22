using System;
using System.IO;

namespace AHPP_AI.Debug
{
    /// <summary>
    ///     Handles logging to both console and file with thread safety
    /// </summary>
    public class Logger
    {
        private readonly bool consoleLogging = true;
        private readonly string filePath;
        private readonly object lockObject = new object();

        /// <summary>
        ///     Initializes a new logger that writes to the specified file
        /// </summary>
        /// <param name="filePath">Path to the log file</param>
        /// <param name="enableConsole">Whether to output to console as well</param>
        public Logger(string filePath, bool enableConsole = true)
        {
            this.filePath = filePath;
            consoleLogging = enableConsole;

            try
            {
                // Clear previous log file
                File.WriteAllText(filePath, $"=== Log started at {DateTime.Now} ===\r\n");
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
            var logMessage = $"{timestamp} [{level}] {message}";

            // Output to console if enabled
            if (consoleLogging)
            {
                Console.ForegroundColor = GetColorForLevel(level);
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
        ///     Get console color for different log levels
        /// </summary>
        private ConsoleColor GetColorForLevel(string level)
        {
            switch (level.ToUpper())
            {
                case "ERROR":
                    return ConsoleColor.Red;
                case "WARN":
                    return ConsoleColor.Yellow;
                case "DEBUG":
                    return ConsoleColor.Cyan;
                default:
                    return ConsoleColor.Gray;
            }
        }
    }
}
