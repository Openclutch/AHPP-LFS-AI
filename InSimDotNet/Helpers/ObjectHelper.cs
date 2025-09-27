using System.Collections.Generic;

namespace InSimDotNet.Helpers
{
    /// <summary>
    ///     Static class to help with object names.
    /// </summary>
    public static class ObjectHelper
    {
        private static readonly Dictionary<int, string[]> ObjMap = new Dictionary<int, string[]>
        {
            { 0, new[] { "Unknown", "Unknown Object" } },

            { 4, new[] { "Chalk", "Chalk Line Long" } },
            { 5, new[] { "Chalk", "Chalk Line" } },
            { 6, new[] { "Chalk", "Chalk Ahead" } },
            { 7, new[] { "Chalk", "Chalk Ahead Long" } },
            { 8, new[] { "Chalk", "Chalk Soft Left" } },
            { 9, new[] { "Chalk", "Chalk Hard Left" } },
            { 10, new[] { "Chalk", "Chalk Soft Left Long" } },
            { 11, new[] { "Chalk", "Chalk Soft Right" } },
            { 12, new[] { "Chalk", "Chalk Hard Right" } },
            { 13, new[] { "Chalk", "Chalk Soft Right Long" } },

            { 20, new[] { "Cone", "Cone Red/White" } },
            { 21, new[] { "Cone", "Cone Red" } },
            { 22, new[] { "Cone", "Cone Red Striped" } },
            { 23, new[] { "Cone", "Cone Blue Striped" } },
            { 24, new[] { "Cone", "Cone Blue" } },
            { 25, new[] { "Cone", "Cone Green Striped" } },
            { 26, new[] { "Cone", "Cone Green" } },
            { 27, new[] { "Cone", "Cone Orange" } },
            { 28, new[] { "Cone", "Cone White" } },
            { 29, new[] { "Cone", "Cone Yellow Striped" } },
            { 30, new[] { "Cone", "Cone Yellow" } },

            { 40, new[] { "Cone", "Cone Red Directional" } },
            { 41, new[] { "Cone", "Cone Blue Directional" } },
            { 42, new[] { "Cone", "Cone Green Directional" } },
            { 43, new[] { "Cone", "Cone Yellow Directional" } },

            { 48, new[] { "Tyre", "Tyre" } },
            { 49, new[] { "Tyres", "Tyre Stack of 2" } },
            { 50, new[] { "Tyres", "Tyre Stack of 3" } },
            { 51, new[] { "Tyres", "Tyre Stack of 4" } },
            { 52, new[] { "Tyre", "Big Tyre" } },
            { 53, new[] { "Tyres", "Big Tyre Stack of 2" } },
            { 54, new[] { "Tyres", "Big Tyre Stack of 3" } },
            { 55, new[] { "Tyres", "Big Tyre Stack of 4" } },

            { 64, new[] { "Marker", "Marker Curve Left" } },
            { 65, new[] { "Marker", "Marker Curve Right" } },
            { 66, new[] { "Marker", "Marker Left" } },
            { 67, new[] { "Marker", "Marker Right" } },
            { 68, new[] { "Marker", "Marker Left Hard" } },
            { 69, new[] { "Marker", "Marker Right Hard" } },
            { 70, new[] { "Marker", "Marker Left->Right" } },
            { 71, new[] { "Marker", "Marker Right->Left" } },
            { 72, new[] { "Marker", "Marker U-Turn->Right" } },
            { 73, new[] { "Marker", "Marker U-Turn->Left" } },
            { 74, new[] { "Marker", "Marker Winding Left" } },
            { 75, new[] { "Marker", "Marker Winding Right" } },
            { 76, new[] { "Marker", "Marker U-Turn Left" } },
            { 77, new[] { "Marker", "Marker U-Turn Right" } },

            { 84, new[] { "Marker", "Marker 25" } },
            { 85, new[] { "Marker", "Marker 50" } },
            { 86, new[] { "Marker", "Marker 75" } },
            { 87, new[] { "Marker", "Marker 100" } },
            { 88, new[] { "Marker", "Marker 125" } },
            { 89, new[] { "Marker", "Marker 150" } },
            { 90, new[] { "Marker", "Marker 200" } },
            { 91, new[] { "Marker", "Marker 250" } },

            { 96, new[] { "Railing", "Railing Short" } },
            { 97, new[] { "Railing", "Railing Medium" } },
            { 98, new[] { "Railing", "Railing Long" } },

            { 104, new[] { "Barrier", "Barrier Long" } },
            { 105, new[] { "Barrier", "Barrier Red" } },
            { 106, new[] { "Barrier", "Barrier White" } },

            { 112, new[] { "Banner", "Banner 1" } },
            { 113, new[] { "Banner", "Banner 2" } },

            { 120, new[] { "Ramp", "Ramp" } },
            { 121, new[] { "Ramp", "Ramp Wide" } },

            { 128, new[] { "Speed Bump", "Speed Bump Long" } },
            { 129, new[] { "Speed Bump", "Speed Bump" } },

            { 136, new[] { "Post", "Post Green" } },
            { 137, new[] { "Post", "Post Orange" } },
            { 138, new[] { "Post", "Post Red" } },
            { 139, new[] { "Post", "Post White" } },

            { 144, new[] { "Bale", "Bale" } },

            { 148, new[] { "Railing", "Railing" } },
            { 149, new[] { "Control", "Start lights" } },

            { 160, new[] { "Sign", "Sign Keep Left" } },
            { 161, new[] { "Sign", "Sign Keep Right" } },

            { 168, new[] { "Sign", "Sign 80 km/h" } },
            { 169, new[] { "Sign", "Sign 50 km/h" } },

            { 172, new[] { "Concrete", "Concrete Slab" } },
            { 173, new[] { "Concrete", "Concrete Ramp" } },
            { 174, new[] { "Concrete", "Concrete Wall" } },
            { 175, new[] { "Concrete", "Concrete Pillar" } },
            { 176, new[] { "Concrete", "Concrete Slab Wall" } },
            { 177, new[] { "Concrete", "Concrete Ramp Wall" } },
            { 178, new[] { "Concrete", "Concrete Short Slab Wall" } },
            { 179, new[] { "Concrete", "Concrete Wedge" } },

            { 184, new[] { "Control", "Start position" } },
            { 185, new[] { "Control", "Pit Start Point" } },
            { 186, new[] { "Control", "Pit Stop Box" } },

            { 254, new[] { "Marshall", "Restricted area" } },
            { 255, new[] { "Marshall", "Route checker" } }
        };

        /// <summary>
        ///     Determines the full name of an object or null if the index does not exist.
        /// </summary>
        /// <param name="index">The object's index.</param>
        /// <returns>The full name.</returns>
        public static string GetObjName(int index)
        {
            string[] obj;
            if (ObjMap.TryGetValue(index, out obj)) return obj[1];
            return null;
        }

        /// <summary>
        ///     Determines the type of the object or returns null if the index does not exist.
        /// </summary>
        /// <param name="index">The object's index.</param>
        /// <returns>The type of the object.</returns>
        public static string GetObjType(int index)
        {
            string[] obj;
            if (ObjMap.TryGetValue(index, out obj)) return obj[0];
            return null;
        }

        /// <summary>
        ///     Determines if the specified object exists.
        /// </summary>
        /// <param name="index">The index of the object.</param>
        /// <returns>True if the object exists.</returns>
        public static bool ObjExists(int index)
        {
            return ObjMap.ContainsKey(index);
        }
    }
}