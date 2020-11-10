#ifndef __XPCUDPSOCK___
#define __XPCUDPSOCK___

#pragma comment(lib,"Ws2_32.lib")
#pragma comment(lib,"xPCUDPVS9.lib")

// Call this function to initialize. This should be done before instantiating any CUDPReceiver or CUDPSender.
BOOL InitUDPLib();

class CUDPReceiver
{
public:
        /* Default constructor for CUDPReceiver
         * Params 
         *    size: size of the packet
         *    port: port number to listen to. */
	CUDPReceiver(UINT size, UINT port);

	/* destructor */
	~CUDPReceiver();

        /* Get latest incoming data to a buffer 
         * Params
         *    data: pointer to the buffer */
	void GetData(void* data);

private:
	SOCKET m_socket;
	sockaddr_in m_addr;
	int m_addrSize;


	UINT m_packageSize;
	char* m_buf;
};


class CUDPSender
{
public:
        /* Default constructor for CUDPSender
         * Params 
         *    size: size of the packet
         *    port: port number to send to. 
         *    address: string containing the address of the destination. */
	CUDPSender(UINT size, UINT port, const char* address);
    
	/* destructor */
	~CUDPSender();

        /* Get data in a buffer out to the destination
         * Params
         *    data: pointer to the buffer */
	void SendData(void* data);

private:
	SOCKET m_socket;
	sockaddr_in m_addr;	
	UINT m_packageSize;
};


#endif //__XPCUDPSOCK___