using InSimDotNet.Packets;

namespace InSimDotNet
{
    /// <summary>
    ///     Provides a read-only representation of the <see cref="InSimSettings" /> class.
    /// </summary>
    public class ReadOnlyInSimSettings
    {
        private readonly InSimSettings settings;

        /// <summary>
        ///     Creates a new instance of the  <see cref="ReadOnlyInSimSettings" /> class.
        /// </summary>
        /// <param name="settings">The InSimSettings to make readonly.</param>
        public ReadOnlyInSimSettings(InSimSettings settings)
        {
            this.settings = settings;
        }

        /// <summary>
        ///     Gets the address of the remote host.
        /// </summary>
        public string Host => settings.Host;

        /// <summary>
        ///     Gets the port of the remote host.
        /// </summary>
        public int Port => settings.Port;

        /// <summary>
        ///     Gets the UDP port to use for <see cref="IS_MCI" /> and <see cref="IS_NLP" /> packet updates. If set a
        ///     separate UDP connection will be opened on that port.
        /// </summary>
        public int UdpPort => settings.UdpPort;

        /// <summary>
        ///     Gets the InSim initialization flags.
        /// </summary>
        public InSimFlags Flags => settings.Flags;

        /// <summary>
        ///     Gets the hidden host command prefix.
        /// </summary>
        public char Prefix => settings.Prefix;

        /// <summary>
        ///     Gets the number of milliseconds between <see cref="IS_MCI" /> or <see cref="IS_NLP" /> packets.
        /// </summary>
        public int Interval => settings.Interval;

        /// <summary>
        ///     Gets the LFS game admin password.
        /// </summary>
        public string Admin => settings.Admin;

        /// <summary>
        ///     Gets a short name for the program.
        /// </summary>
        public string IName => settings.IName;

        /// <summary>
        ///     Gets if the host is an InSim Relay host. If true all other settings are ignored.
        /// </summary>
        public bool IsRelayHost => settings.IsRelayHost;
    }
}