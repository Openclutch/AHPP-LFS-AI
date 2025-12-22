using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;

namespace InSimDotNet
{
    /// <summary>
    /// Handles converting strings from LFS encoding into unicode and vice versa.
    /// </summary>
    public class LfsUnicodeEncoding : LfsEncoding
    {
        private const char ControlChar = '^';
        private const char FallbackChar = '?';
        private static readonly bool IsRunningOnMono = (Type.GetType("Mono.Runtime") != null);
        private static readonly Lazy<Dictionary<char, Encoding>> EncodingMapLazy =
            new Lazy<Dictionary<char, Encoding>>(CreateEncodingMap, System.Threading.LazyThreadSafetyMode.ExecutionAndPublication);

        /// <summary>
        /// Optional callback invoked when a character cannot be encoded with the active codepage.
        /// </summary>
        public static Action<string> DiagnosticsLogger { get; set; }

        private static Dictionary<char, Encoding> EncodingMap => EncodingMapLazy.Value;
        private static readonly Lazy<Encoding> DefaultEncodingLazy =
            new Lazy<Encoding>(() => EncodingMap['L'], System.Threading.LazyThreadSafetyMode.ExecutionAndPublication);
        private static Encoding DefaultEncoding => DefaultEncodingLazy.Value;

        private static readonly Dictionary<char, string> EscapeMap = new Dictionary<char, string> {
            { 'v', "|" },
            { 'a', "*" },
            { 'c', ":" },
            { 'd', "\\" },
            { 's', "/" },
            { 'q', "?" },
            { 't', "\"" },
            { 'l', "<" },
            { 'r', ">" },
            { '^', "^^" }
        };

        /// <summary>
        /// Converts a LFS encoded string to unicode.
        /// </summary>
        /// <param name="buffer">The buffer containing the packet data.</param>
        /// <param name="index">The index that the string starts in the packet data.</param>
        /// <param name="length">The length of the string.</param>
        /// <returns>The resulting unicode string.</returns>
        public override string GetString(byte[] buffer, int index, int length)
        {
            StringBuilder output = new StringBuilder(length);
            Encoding encoding = DefaultEncoding;
            Encoding nextEncoding;
            int i = 0, start = index;
            string escape;

            for (i = index; i < index + length; i++)
            {
                char control = (char)buffer[i];

                // Check for null terminator.
                if (control == Char.MinValue)
                {
                    break;
                }

                // If not control character then ignore.
                if (control != ControlChar)
                {
                    continue;
                }

                // Found control character so encode everything up to this point.
                if (i - start > 0)
                {
                    output.Append(encoding.GetString(buffer, start, (i - start)));
                }
                start = (i + 2); // skip control chars.

                // Process control character.
                char next = (char)buffer[++i];
                if (EncodingMap.TryGetValue(next, out nextEncoding))
                {
                    encoding = nextEncoding; // Switch encoding.
                }
                else if (EscapeMap.TryGetValue(next, out escape))
                {
                    output.Append(escape); // Escape character.
                }
                else
                {
                    // Character not codepage switch or escape, so just ignore it.
                    output.Append(control);
                    output.Append(next);
                }
            }

            // End of string reached so encode up all to this point.
            if (i - start > 0)
            {
                output.Append(encoding.GetString(buffer, start, (i - start)));
            }

            return output.ToString();
        }

        /// <summary>
        /// Converts a unicode string into a LFS encoded string.
        /// </summary>
        /// <param name="value">The unicode string to convert.</param>
        /// <param name="buffer">The packet buffer into which the bytes will be written.</param>
        /// <param name="index">The index in the packet buffer to start writing bytes.</param>
        /// <param name="length">The maximum number of bytes to write.</param>
        /// <returns>The number of bytes written during the operation.</returns>
        public override int GetBytes(string value, byte[] buffer, int index, int length)
        {
            Encoding encoding = DefaultEncoding;
            byte[] tempBytes = new byte[4];
            int tempCount;
            int start = index;
            int totalLength = index + (length - 1);

            for (int i = 0; i < value.Length && index < totalLength; i++)
            {
                // Remove any existing language tags from the string.
                int next = i + 1;
                if (value[i] == ControlChar && next < value.Length)
                {
                    var controlCode = value[next];
                    if (EncodingMap.TryGetValue(controlCode, out var explicitEncoding))
                    {
                        encoding = explicitEncoding;

                        // Ensure there is space to emit the control code before continuing.
                        if (index + 1 >= totalLength)
                        {
                            break;
                        }

                        buffer[index++] = (byte)ControlChar;
                        buffer[index++] = (byte)controlCode;
                        i++; // skip codepage char
                        continue;
                    }
                }

                if (value[i] <= 127)
                {
                    // All codepages share ASCII values.
                    buffer[index++] = (byte)value[i];
                }
                else if (TryGetBytes(encoding, value[i], tempBytes, out tempCount))
                {
                    // Character exists in current codepage.
                    Buffer.BlockCopy(tempBytes, 0, buffer, index, tempCount);
                    index += tempCount;
                }
                else
                {
                    // Search for new codepage.
                    bool found = false;
                    Encoding attemptedEncoding = encoding;
                    char? attemptedEncodingKey = GetEncodingKey(encoding);
                    Encoding replacementEncoding = null;
                    char? replacementEncodingKey = null;

                    foreach (KeyValuePair<char, Encoding> map in EncodingMap)
                    {
                        if (map.Value == encoding)
                        {
                            continue; // Skip current as we've already searched it.
                        }

                        if (TryGetBytes(map.Value, value[i], tempBytes, out tempCount))
                        {
                            // Switch codepage.
                            encoding = map.Value;

                            // Add control characters.
                            buffer[index++] = (byte)ControlChar;
                            buffer[index++] = (byte)map.Key;

                            // Copy to buffer.
                            Buffer.BlockCopy(tempBytes, 0, buffer, index, tempCount);
                            index += tempCount;
                            found = true;
                            replacementEncoding = map.Value;
                            replacementEncodingKey = map.Key;

                            break;
                        }
                    }

                    LogUnsupportedCharacter(
                        value[i],
                        attemptedEncoding,
                        attemptedEncodingKey,
                        found,
                        replacementEncoding,
                        replacementEncodingKey);

                    // If not found in any codepage then add fallback character.
                    if (!found)
                    {
                        buffer[index++] = (byte)FallbackChar;
                    }
                }
            }

            return index - start;
        }

        /// <summary>
        /// Tries to convert a unicode character into a LFS encoded one.
        /// </summary>
        /// <param name="encoding">The encoding to attempt to convert the character into.</param>
        /// <param name="value">The character to attempt to encode.</param>
        /// <param name="bytes">The array to write the LFS encoded character into.</param>
        /// <param name="count">The number of bytes that the character was encoded into.</param>
        /// <returns>Returns true if the conversion was successful or false if otherwise.</returns>
        [DebuggerStepThrough]
        private static bool TryGetBytes(Encoding encoding, char value, byte[] bytes, out int count)
        {
            // We use WideCharToMultiByte on Windows as it's very fast, but that's not 
            // available on Mono so we revert to trying to convert a character and then 
            // catching the exception generated when it fails. This is very slow as the 
            // callstack may potentionally need to be unwound for every character in the 
            // string.
            if (IsRunningOnMono)
            {
                return TryGetBytesMono(encoding, value, bytes, out count);
            }

            return TryGetBytesDotNet(encoding, value, bytes, out count);
        }

        private static bool TryGetBytesDotNet(Encoding encoding, char value, byte[] bytes, out int count)
        {
            count = encoding.GetBytes(value.ToString(), 0, 1, bytes, 0);
            if (UsedReplacementFallback(bytes, count))
            {
                count = 0;
                return false;
            }

            return true;
        }

        private static bool TryGetBytesMono(Encoding encoding, char value, byte[] bytes, out int count)
        {
            count = encoding.GetBytes(value.ToString(), 0, 1, bytes, 0);
            if (UsedReplacementFallback(bytes, count))
            {
                count = 0;
                return false;
            }

            return true;
        }

        /// <summary>
        /// Logs diagnostic details about characters that cannot be encoded with the active codepage.
        /// </summary>
        /// <param name="value">The character that failed to encode.</param>
        /// <param name="attemptedEncoding">The encoding that failed.</param>
        /// <param name="attemptedEncodingKey">The Insim control code used to enter the attempted encoding.</param>
        /// <param name="switchedEncoding">Flag indicating whether an alternative encoding was found.</param>
        /// <param name="replacementEncoding">The replacement encoding that succeeded, if any.</param>
        /// <param name="replacementEncodingKey">The Insim control code for the replacement encoding, if any.</param>
        private static void LogUnsupportedCharacter(
            char value,
            Encoding attemptedEncoding,
            char? attemptedEncodingKey,
            bool switchedEncoding,
            Encoding replacementEncoding,
            char? replacementEncodingKey)
        {
            if (switchedEncoding)
            {
                // Successful codepage switches are routine (e.g. ☆ requires ^J) so suppress noisy warnings.
                return;
            }

            var attemptedCodePage = attemptedEncoding?.CodePage;
            var message =
                $"Character \"{DescribeChar(value)}\" (U+{(int)value:X4}) not encodable in code page {attemptedCodePage}{FormatEncodingTag(attemptedEncodingKey)}; falling back to \"{FallbackChar}\".";

            WriteDiagnostics(message);
        }

        private static void WriteDiagnostics(string message)
        {
            var logger = DiagnosticsLogger;
            if (logger != null)
            {
                try
                {
                    logger(message);
                    return;
                }
                catch
                {
                    // Swallow logging exceptions to avoid cascading failures.
                }
            }

            Trace.TraceWarning(message);
        }

        private static string DescribeChar(char value)
        {
            if (char.IsControl(value) || char.IsWhiteSpace(value))
            {
                return $"\\u{(int)value:X4}";
            }

            return value.ToString();
        }

        private static string FormatEncodingTag(char? encodingKey)
        {
            return encodingKey.HasValue ? $" (^{encodingKey.Value})" : string.Empty;
        }

        private static char? GetEncodingKey(Encoding encoding)
        {
            foreach (var map in EncodingMap)
            {
                if (map.Value == encoding)
                {
                    return map.Key;
                }
            }

            return null;
        }

        /// <summary>
        /// Builds the encoding map while ensuring legacy code pages are registered first.
        /// </summary>
        /// <returns>
        /// Mapping of Insim control codes to the corresponding <see cref="Encoding"/> instances.
        /// </returns>
        private static Dictionary<char, Encoding> CreateEncodingMap()
        {
            LfsEncoding.Initialize();

            // ExceptionFallback is only used by Mono code path, otherwise we want ReplacementFallback everywhere.
            // These codepages don't translate perfectly to LFS but are the best mirror available in .NET.
            return new Dictionary<char, Encoding>
            {
                { 'L', CreateEncodingWithReplacementFallback(1252) },
                { 'G', CreateEncodingWithReplacementFallback(1253) },
                { 'C', CreateEncodingWithReplacementFallback(1251) },
                { 'J', CreateEncodingWithReplacementFallback(932) },
                { 'E', CreateEncodingWithReplacementFallback(1250) },
                { 'T', CreateEncodingWithReplacementFallback(1254) },
                { 'B', CreateEncodingWithReplacementFallback(1257) },
                { 'H', CreateEncodingWithReplacementFallback(950) },
                { 'S', CreateEncodingWithReplacementFallback(936) },
                { 'K', CreateEncodingWithReplacementFallback(949) },
            };
        }

        private static Encoding CreateEncodingWithReplacementFallback(int codePage)
        {
            var encoding = (Encoding)Encoding.GetEncoding(codePage, EncoderFallback.ReplacementFallback, DecoderFallback.ReplacementFallback).Clone();
            encoding.EncoderFallback = new EncoderReplacementFallback("??");
            encoding.DecoderFallback = DecoderFallback.ReplacementFallback;
            return encoding;
        }

        private static bool UsedReplacementFallback(byte[] bytes, int count)
        {
            if (count < 2)
            {
                return false;
            }

            for (int i = 0; i < count; i++)
            {
                if (bytes[i] != (byte)'?')
                {
                    return false;
                }
            }

            return true;
        }
    }
}
