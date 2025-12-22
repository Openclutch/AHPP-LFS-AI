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
        private readonly Dictionary<string, Dictionary<string, string>> sections =
            new Dictionary<string, Dictionary<string, string>>(StringComparer.OrdinalIgnoreCase);

        private AppConfig()
        {
        }

        public static AppConfig Load(string path, Logger logger = null)
        {
            var config = new AppConfig();
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
    }
}
