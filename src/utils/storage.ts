/**
 * Compresses a string using GZIP via the Compression Streams API and returns a Base64 string.
 */
export async function compressData(data: string): Promise<string> {
  const encoder = new TextEncoder();
  const bytes = encoder.encode(data);
  const stream = new Blob([bytes]).stream();
  const compressedStream = stream.pipeThrough(new CompressionStream('gzip'));
  const compressedResponse = new Response(compressedStream);
  const compressedBytes = await compressedResponse.arrayBuffer();
  
  // Convert ArrayBuffer to Base64 safely
  const bytesArray = new Uint8Array(compressedBytes);
  let binary = '';
  for (let i = 0; i < bytesArray.byteLength; i++) {
    binary += String.fromCharCode(bytesArray[i]);
  }
  return btoa(binary);
}

/**
 * Decompresses a GZIP-encoded Base64 string back to its original string.
 */
export async function decompressData(base64: string): Promise<string> {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i++) {
    bytes[i] = binary.charCodeAt(i);
  }
  
  const stream = new Blob([bytes]).stream();
  const decompressedStream = stream.pipeThrough(new DecompressionStream('gzip'));
  const decompressedResponse = new Response(decompressedStream);
  const decompressedBytes = await decompressedResponse.arrayBuffer();
  
  const decoder = new TextDecoder();
  return decoder.decode(decompressedBytes);
}
