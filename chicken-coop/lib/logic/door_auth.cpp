#include "door_auth.h"

#include <cctype>
#include <cstring>

namespace logic
{
    bool isAuthorizedDoorRequest(const char *requestLine, const char *expectedToken)
    {
        if (requestLine == nullptr || expectedToken == nullptr)
        {
            return false;
        }
        // token never configured -> endpoint disabled
        if (expectedToken[0] == '\0' || strcmp(expectedToken, "unset") == 0)
        {
            return false;
        }

        static const char prefix[] = "GET /open?token=";
        const size_t prefixLen = sizeof(prefix) - 1;
        if (strncmp(requestLine, prefix, prefixLen) != 0)
        {
            return false;
        }

        // token ends at the first space (" HTTP/1.1") or at end of line
        const char *token = requestLine + prefixLen;
        const char *end = token;
        while (*end != '\0' && *end != ' ')
        {
            end++;
        }
        // trim trailing whitespace ('\r' left by readStringUntil('\n'))
        while (end > token && isspace(static_cast<unsigned char>(end[-1])))
        {
            end--;
        }

        const size_t tokenLen = static_cast<size_t>(end - token);
        return tokenLen == strlen(expectedToken) &&
               strncmp(token, expectedToken, tokenLen) == 0;
    }
}
