#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include <fstream>

using namespace ns3;
using std::cout;
using std::endl;

NS_LOG_COMPONENT_DEFINE("LeoProj");

static std::ofstream g_output;
static double g_startTime = 0;
static double g_endTime = 0;
static double g_leoSendTime = 0; 

class SourceApp : public Application
{
public:
    SourceApp() {}
    virtual ~SourceApp() {}

    void Setup(Ptr<Socket> socket, Address peer, uint32_t pktSize)
    {
        m_socket = socket;
        m_peer = peer;
        m_pktSize = pktSize;
    }

private:
    virtual void StartApplication()
    {
        m_socket->Connect(m_peer);
        Simulator::Schedule(Seconds(0.1), &SourceApp::SendPacket, this);
    }

    void SendPacket()
    {
        Ptr<Packet> pkt = Create<Packet>(m_pktSize);
        m_socket->Send(pkt);
    }

    Ptr<Socket> m_socket;
    Address m_peer;
    uint32_t m_pktSize;
};

class LEOApp : public Application
{
public:
    LEOApp() {}
    virtual ~LEOApp() {}

    void Setup(Ptr<Socket> recvSock, Ptr<Socket> sendSock, double ratio, uint32_t delayMs)
    {
        m_recvSock = recvSock;
        m_sendSock = sendSock;
        m_ratio = ratio;
        m_delayMs = delayMs;
    }

private:
    virtual void StartApplication()
    {
        m_recvSock->SetRecvCallback(MakeCallback(&LEOApp::ReceivePacket, this));
    }

    void ReceivePacket(Ptr<Socket> socket)
    {
        Ptr<Packet> pkt;
        while ((pkt = socket->Recv()))
        {
            Simulator::Schedule(MilliSeconds(m_delayMs), &LEOApp::SendCompressed, this, pkt);
        }
    }

    void SendCompressed(Ptr<Packet> pkt)
    {
        uint32_t newSize = static_cast<uint32_t>(pkt->GetSize() * m_ratio);
        Ptr<Packet> compressed = Create<Packet>(newSize);
        g_leoSendTime = Simulator::Now().GetSeconds(); // 記錄壓縮完成時間
        m_sendSock->Send(compressed);
        m_totalCompressedBits += newSize * 8;
    }

public:
    double GetTotalCompressedBits() const { return m_totalCompressedBits; }

private:
    Ptr<Socket> m_recvSock;
    Ptr<Socket> m_sendSock;
    double m_ratio;
    uint32_t m_delayMs;
    double m_totalCompressedBits = 0; // 下行 bits
};

void GroundReceive(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    if ((pkt = socket->Recv()))
    {
        g_endTime = Simulator::Now().GetSeconds();
    }
}

double CalculatePropDelay(Ptr<Node> a, Ptr<Node> b)
{
    Vector posA = a->GetObject<MobilityModel>()->GetPosition();
    Vector posB = b->GetObject<MobilityModel>()->GetPosition();
    double distance = (posB - posA).GetLength(); // m
    return distance / 3e8; 
}

void RunExperiment(double ratio, uint32_t delayMode, uint32_t pktSize)
{
    NodeContainer nodes;
    nodes.Create(3); // Source, LEO, Ground

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(0, 0, 0));       // nycu
    posAlloc->Add(Vector(0, 1000, 600e3));  // LEO 高度 600 km
    posAlloc->Add(Vector(0, 270e3, 0));         // kaohsiung
    MobilityHelper mobility;
    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // 計算 propagation delay
    double delaySrcLEO = CalculatePropDelay(nodes.Get(0), nodes.Get(1));
    double delayLEOGnd = CalculatePropDelay(nodes.Get(1), nodes.Get(2));

    // Source -> LEO
    PointToPointHelper p2p1;
    p2p1.SetDeviceAttribute("DataRate", StringValue("100Kbps"));
    p2p1.SetChannelAttribute("Delay", TimeValue(Seconds(delaySrcLEO)));
    NetDeviceContainer dev1 = p2p1.Install(nodes.Get(0), nodes.Get(1));

    // LEO -> Ground
    PointToPointHelper p2p2;
    p2p2.SetDeviceAttribute("DataRate", StringValue("100Kbps"));
    p2p2.SetChannelAttribute("Delay", TimeValue(Seconds(delayLEOGnd)));
    NetDeviceContainer dev2 = p2p2.Install(nodes.Get(1), nodes.Get(2));

    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer if1 = ipv4.Assign(dev1);
    ipv4.SetBase("10.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer if2 = ipv4.Assign(dev2);

    // Source socket
    Ptr<Socket> sourceSock = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Address leoAddr = InetSocketAddress(if1.GetAddress(1), 8080);
    Ptr<SourceApp> sourceApp = CreateObject<SourceApp>();
    sourceApp->Setup(sourceSock, leoAddr, pktSize);
    nodes.Get(0)->AddApplication(sourceApp);
    sourceApp->SetStartTime(Seconds(1.0));
    sourceApp->SetStopTime(Seconds(2.0));

    // LEO socket
    Ptr<Socket> leoRecvSock = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    leoRecvSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
    Ptr<Socket> leoSendSock = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    Address groundAddr = InetSocketAddress(if2.GetAddress(1), 8080);
    leoSendSock->Connect(groundAddr);

    uint32_t compressDelayMs = (delayMode == 0) ? 0 : static_cast<uint32_t>(5.0 * (pktSize / 1000.0) / ratio);
    Ptr<LEOApp> leoApp = CreateObject<LEOApp>();
    leoApp->Setup(leoRecvSock, leoSendSock, ratio, compressDelayMs);
    nodes.Get(1)->AddApplication(leoApp);
    leoApp->SetStartTime(Seconds(1.0));
    leoApp->SetStopTime(Seconds(40.0));

    // Ground socket
    Ptr<Socket> groundSock = Socket::CreateSocket(nodes.Get(2), UdpSocketFactory::GetTypeId());
    groundSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8080));
    groundSock->SetRecvCallback(MakeCallback(&GroundReceive));

    g_startTime = 1.0; 
    g_endTime = 0;
    g_leoSendTime = 0;

    Simulator::Stop(Seconds(50.0));
    Simulator::Run();
    Simulator::Destroy();

    // throughput
    double upBits = pktSize * 8;
    double upTime = g_leoSendTime - g_startTime; // Source→LEO 
    double upThroughput = upBits / upTime / 1e6; // Mbps

    double downBits = leoApp->GetTotalCompressedBits();
    double downTime = g_endTime - g_leoSendTime; // LEO→Ground 
    double downThroughput = downBits / downTime / 1e6; // Mbps

    double totalTime = g_endTime - g_startTime;

    g_output << ratio << "," << delayMode << "," << pktSize << ","
             << compressDelayMs << ","
             << upThroughput << "," << downThroughput << ","
             << totalTime << endl;

    cout << "Ratio=" << ratio
         << " DelayMode=" << delayMode
         << " Pkt(byte)=" << pktSize
         << " CompressDelay(ms)=" << compressDelayMs
         << " Up(Mbps)=" << upThroughput
         << " Down(Mbps)=" << downThroughput
         << " TotalTime(s)=" << totalTime
         << endl;
}

int main(int argc, char *argv[])
{
    g_output.open("leo-results.csv");
    g_output << "CompressionRatio,DelayMode,PacketSize(byte),CompressDelay(ms),UpThroughput(Mbps),DownThroughput(Mbps),TotalTime(s)" << endl;

    double ratios[] = {1.0, 0.5, 0.2};
    uint32_t delays[] = {0, 1}; 
    uint32_t pkts[] = {1000, 5000, 10000};

    for (auto r : ratios)
    {
        for (auto d : delays)
        {
            for (auto p : pkts)
            {
                RunExperiment(r, d, p);
            }
        }
    }

    g_output.close();
    cout << "All experiments completed." << endl;
    return 0;
}
