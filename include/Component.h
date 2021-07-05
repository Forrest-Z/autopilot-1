
#ifndef	__INCCOMPONENTH
#define __INCCOMPONENTH

#ifdef __cplusplus
extern "C" {
#endif

/*define component struct */
typedef	struct base_component
{
	#include "BaseComponent.h"
}Component;

#ifdef __cplusplus
}
#endif

#endif //__INCCOMPONENTH

