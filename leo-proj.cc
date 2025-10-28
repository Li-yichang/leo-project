#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include <fstream>
#include <vector>

using namespace ns3;
using std::cout;
using std::endl;

NS_LOG_COMPONENT_DEFINE("LeoProj");

static std::ofstream g_output;
static uint32_t g_recvCount = 0;
static double g_startTime = 0;
static double g_endTime = 0;

class CompressorApp : public Application
{
public:
    CompressorApp() {}
    virtual ~CompressorApp() {}

    void Setup(Ptr<Socket> socket, Address address, uint32_t pktSize, double ratio, uint32_t delayMs)
    {
        m_socket = socket;
        m_peerAddress = address;
        m_pktSize = pktSize;
        m_ratio = ratio;
        m_delayMs = delayMs;
    }

private:
    virtual void StartApplication()
    {
        m_socket->Connect(m_peerAddress);
        Simulator::Schedule(Seconds(0.1), &CompressorApp::SendPacket, this);
    }

    virtual void StopApplication()
    {
        if (m_socket)
            m_socket->Close();
    }

    void SendPacket()
    {
        Ptr<Packet> pkt = Create<Packet>(m_pktSize);

        // 模擬處理延遲後再壓縮
        Simulator::Schedule(MilliSeconds(m_delayMs), &CompressorApp::DelayedSend, this, pkt);

        // 每 n 秒發送一個封包
        double interval = (m_pktSize * m_ratio * 8) / (100 * 1000); // 秒
        Simulator::Schedule(Seconds(interval), &CompressorApp::SendPacket, this);


    }

    void DelayedSend(Ptr<Packet> pkt)
    {
        uint32_t newSize = static_cast<uint32_t>(pkt->GetSize() * m_ratio);
        Ptr<Packet> compressed = Create<Packet>(newSize);
        m_socket->Send(compressed);
    }

private:
    Ptr<Socket> m_socket;
    Address m_peerAddress;
    uint32_t m_pktSize;
    double m_ratio;
    uint32_t m_delayMs;
};

void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    while ((pkt = socket->Recv()))
    {
        if (g_recvCount == 0)
            g_startTime = Simulator::Now().GetSeconds();
        g_recvCount++;
        g_endTime = Simulator::Now().GetSeconds();
    }
}

void RunOneExperiment(uint32_t sats, double ratio, uint32_t delayMs, uint32_t pktSize)
{
    NodeContainer nodes;
    nodes.Create(2);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Kbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1000ms"));
    NetDeviceContainer dev = p2p.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ifc = ipv4.Assign(dev);

    Ptr<Socket> recvSocket = Socket::CreateSocket(nodes.Get(1), UdpSocketFactory::GetTypeId());
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 8080);
    recvSocket->Bind(local);
    recvSocket->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> sendSocket = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());
    Address remote = InetSocketAddress(ifc.GetAddress(1), 8080);

    Ptr<CompressorApp> app = CreateObject<CompressorApp>();
    app->Setup(sendSocket, remote, pktSize, ratio, delayMs);
    nodes.Get(0)->AddApplication(app);
    app->SetStartTime(Seconds(1.0));
    app->SetStopTime(Seconds(30.0));

    g_recvCount = 0;
    g_startTime = 0;
    g_endTime = 0;

    Simulator::Stop(Seconds(40.0));
    Simulator::Run();
    Simulator::Destroy();

    double totalTime = g_endTime - g_startTime;
    double SingleTime=totalTime/g_recvCount;
    g_output << sats << "," << ratio << "," << delayMs << "," << pktSize << "," << g_recvCount << "," << totalTime << "," << SingleTime << endl;

    cout << "Sats=" << sats
         << " Ratio=" << ratio
         << " Delay=" << delayMs
         << " Pkt=" << pktSize
         << " Count=" << g_recvCount
         << " TotalTime=" << totalTime 
         << " singleTime=" << SingleTime
         << endl;
}

int main(int argc, char *argv[])
{
    g_output.open("leo-results.csv");
    g_output << "Satellites,CompressionRatio,DelayMs,PacketSize,PacketsReceived,TotalTransmissionTime(s),singleTransmissionTime(s)" << endl;

    uint32_t sats[] = {6};
    double ratios[] = {1.0, 0.5, 0.2};
    uint32_t delays[] = {0, 500,10000};
    uint32_t pkts[] = {1000, 5000, 10000};

    for (auto s : sats)
    {
        for (auto r : ratios)
        {
            for (auto d : delays)
            {
                for (auto p : pkts)
                {
                    RunOneExperiment(s, r, d, p);
                }
            }
        }
    }

    g_output.close();
    cout << "All 27 experiments completed." << endl;
    return 0;
}
