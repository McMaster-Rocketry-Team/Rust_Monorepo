import menuIcon from '../../assets/menu.png'

export default function ChannelCard({ ...props }) {
  const name = props.sensorData.name
  return (
    <div className='w-full h-24 bg-[#F5F5F5] rounded-lg p-2'>
      <div className='flex justify-between'>
        <h1 className='text-md'>{name}</h1>
        <div className='h-6 w-6'>
          <img src={menuIcon} alt='' />
        </div>
      </div>
    </div>
  )
}
