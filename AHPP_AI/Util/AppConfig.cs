using System;
using System.Collections.Generic;
using System.IO;
using AHPP_AI.Debug;

namespace AHPP_AI.Util
{
    /// <summary>
    /// Lightweight INI reader to pull configuration values with defaults.
    /// </summary>
    public class AppConfig
    {
        private readonly string filePath;
        private readonly Logger logger;
        private readonly Dictionary<string, Dictionary<string, string>> sections =
            new Dictionary<string, Dictionary<string, string>>(StringComparer.OrdinalIgnoreCase);

        private AppConfig(string filePath, Logger logger)
        {
            this.filePath = filePath;
            this.logger = logger;
        }

        public static AppConfig Load(string path, Logger logger = null)
        {
            var config = new AppConfig(path, logger);
            if (!File.Exists(path))
            {
                logger?.Log($"Config file not found at {path}, using defaults.");
                return config;
            }

            string currentSection = "default";
            config.sections[currentSection] = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

            foreach (var rawLine in File.ReadAllLines(path))
            {
                var line = rawLine.Trim();
                if (string.IsNullOrWhiteSpace(line) || line.StartsWith(";") || line.StartsWith("#"))
                    continue;

                if (line.StartsWith("[") && line.EndsWith("]"))
                {
                    currentSection = line.Substring(1, line.Length - 2).Trim();
                    if (!config.sections.ContainsKey(currentSection))
                        config.sections[currentSection] = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                    continue;
                }

                var kvp = line.Split(new[] { '=' }, 2);
                if (kvp.Length != 2) continue;
                var key = kvp[0].Trim();
                var value = kvp[1].Trim();
                config.sections[currentSection][key] = value;
            }

            return config;
        }

        public string GetString(string section, string key, string defaultValue)
        {
            if (sections.TryGetValue(section, out var entries) && entries.TryGetValue(key, out var value))
                return value;
            return defaultValue;
        }

        public int GetInt(string section, string key, int defaultValue)
        {
            var text = GetString(section, key, null);
            return int.TryParse(text, out var result) ? result : defaultValue;
        }

        public double GetDouble(string section, string key, double defaultValue)
        {
            var text = GetString(section, key, null);
            return double.TryParse(text, out var result) ? result : defaultValue;
        }

        public bool GetBool(string section, string key, bool defaultValue)
        {
            var text = GetString(section, key, null);
            if (string.IsNullOrWhiteSpace(text)) return defaultValue;

            if (int.TryParse(text, out var intVal))
                return intVal != 0;

            return bool.TryParse(text, out var boolVal) ? boolVal : defaultValue;
        }

        /// <summary>
        /// Update a configuration value and optionally persist it to disk.
        /// </summary>
        public void SetString(string section, string key, string value, bool persist = false)
        {
            if (string.IsNullOrWhiteSpace(section) || string.IsNullOrWhiteSpace(key)) return;

            if (!sections.TryGetValue(section, out var entries))
            {
                entries = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                sections[section] = entries;
            }

            entries[key] = value ?? string.Empty;

            if (persist) PersistValue(section, key, value ?? string.Empty);
        }

        /// <summary>
        /// Update a boolean configuration value using numeric conventions (1/0).
        /// </summary>
        public void SetBool(string section, string key, bool value, bool persist = false)
        {
            SetString(section, key, value ? "1" : "0", persist);
        }

        /// <summary>
        /// Persist a specific key/value pair back into the config file, creating sections when missing.
        /// </summary>
        public bool PersistValue(string section, string key, string value)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(filePath)) return false;

                var lines = File.Exists(filePath)
                    ? new List<string>(File.ReadAllLines(filePath))
                    : new List<string>();

                var sectionHeader = $"[{section}]";
                var sectionFound = false;
                var keyUpdated = false;
                var inTargetSection = false;
                var insertIndex = lines.Count;

                for (var i = 0; i < lines.Count; i++)
                {
                    var rawLine = lines[i];
                    var trimmed = rawLine.Trim();

                    if (string.IsNullOrWhiteSpace(trimmed) || trimmed.StartsWith(";") || trimmed.StartsWith("#"))
                        continue;

                    if (trimmed.StartsWith("[") && trimmed.EndsWith("]"))
                    {
                        if (inTargetSection && !keyUpdated)
                        {
                            insertIndex = i;
                            break;
                        }

                        var currentSection = trimmed.Substring(1, trimmed.Length - 2).Trim();
                        inTargetSection = currentSection.Equals(section, StringComparison.OrdinalIgnoreCase);
                        if (inTargetSection)
                        {
                            sectionFound = true;
                            insertIndex = i + 1;
                        }

                        continue;
                    }

                    if (!inTargetSection) continue;

                    var kvp = trimmed.Split(new[] { '=' }, 2);
                    if (kvp.Length != 2) continue;

                    var existingKey = kvp[0].Trim();
                    if (!existingKey.Equals(key, StringComparison.OrdinalIgnoreCase)) continue;

                    lines[i] = $"{key}={value}";
                    keyUpdated = true;
                    break;
                }

                if (!sectionFound)
                {
                    if (lines.Count > 0 && !string.IsNullOrWhiteSpace(lines[^1])) lines.Add(string.Empty);
                    lines.Add(sectionHeader);
                    insertIndex = lines.Count;
                }

                if (!keyUpdated)
                {
                    lines.Insert(insertIndex, $"{key}={value}");
                }

                File.WriteAllLines(filePath, lines);
                return true;
            }
            catch (Exception ex)
            {
                logger?.LogException(ex, $"Failed to persist config value {section}/{key}");
                return false;
            }
        }
    }
}
