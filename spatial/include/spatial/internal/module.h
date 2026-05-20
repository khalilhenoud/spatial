#if !defined(SPATIAL_API)
	#define SPATIAL_API /* NOTHING */

	#if defined(WIN32) || defined(WIN64)
		#undef SPATIAL_API
		#if defined(spatial_EXPORTS)
			#define SPATIAL_API __declspec(dllexport)
		#else
			#define SPATIAL_API __declspec(dllimport)
		#endif
	#endif // defined(WIN32) || defined(WIN64)

#endif // !defined(SPATIAL_API)

