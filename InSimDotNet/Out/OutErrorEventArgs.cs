using System;

namespace InSimDotNet.Out
{
    /// <summary>
    ///     Represents data for the OutError event.
    /// </summary>
    public class OutErrorEventArgs : EventArgs
    {
        /// <summary>
        ///     Creates a new instance of the  <see cref="OutErrorEventArgs" /> class.
        /// </summary>
        /// <param name="exception">The exception associated with the error</param>
        public OutErrorEventArgs(Exception exception)
        {
            Exception = exception;
        }

        /// <summary>
        ///     Gets the exception associated with the error.
        /// </summary>
        public Exception Exception { get; private set; }
    }
}