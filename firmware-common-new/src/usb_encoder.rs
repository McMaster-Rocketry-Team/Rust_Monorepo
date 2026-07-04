// Note , you never need a string larger than 1024 bytes

use heapless::String;

pub fn encode_string(
    buf: &mut [u8],
    index: usize,
    string: String<1024>,
    real_string_size: usize,
) -> Result<(), ()> {
    if index + (real_string_size)  > buf.len() - 1 // index + real_string_size is the last index of the string we write, since we add an extra byte
    {
        return Err(());
    }

    buf[index] = b'S';
    buf[index + 1..index + 1 + real_string_size]
        .copy_from_slice(&string.as_bytes()[0..real_string_size]);

    Ok(())
}

// TODO error checking, what if string doesnt start with S or s, same applies for floats, will help catch errors

// pub fn decode_string(string: String) -> String {
//     let new_string = &string[1..];

//     String::from(new_string)
// }

pub fn encode_float(buf: &mut [u8], float: f32, newline: bool, index: usize) -> Result<(), ()> {
    let bytes = float.to_le_bytes();

    if index + 4 > buf.len() - 1 {
        return Err(());
    }

    buf[index] = if newline { b'F' } else { b'f' };

    buf[index + 1..index + 5].copy_from_slice(&bytes);

    Ok(())
}

pub fn decode_float(data: [u8; 5]) -> Result<f32, ()> {
    if data[0] != b'f' && data[0] != b'F' {
        return Err(());
    }
    let float_data: &[u8; 4] = data[1..].as_array().unwrap();

    Ok(f32::from_le_bytes(*float_data))
}
