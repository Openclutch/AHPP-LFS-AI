using System.Collections.Generic;

namespace AHPP_AI.Waypoint
{
    /// <summary>
    /// Severity level for a route validation issue.
    /// </summary>
    public enum RouteValidationSeverity
    {
        Warning,
        Error
    }

    /// <summary>
    /// Describes a detected problem while validating recorded routes.
    /// </summary>
    public class RouteValidationIssue
    {
        public RouteValidationIssue(RouteValidationSeverity severity, string message)
        {
            Severity = severity;
            Message = message;
        }

        public RouteValidationSeverity Severity { get; }
        public string Message { get; }
    }
}
